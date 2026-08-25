#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_timer.h"

// Hardware UART Interface Configuration
#define UART_PORT       UART_NUM_0
#define UART_BUF_SIZE   256

// Hardware I2C Interface Configuration (Pins 8 & 9 based on diagram.json)
#define I2C_PORT        I2C_NUM_0
#define I2C_SDA         8   
#define I2C_SCL         9   
#define I2C_FREQ        400000

// MPU-6050 Registers & Constants
#define MPU_ADDR        0x68
#define REG_PWR_MGMT    0x6B
#define REG_ACCEL_X     0x3B
#define ACCEL_SCALE     16384.0f

#define CMD_BUF_SIZE    64

// Thread-safe Global Flag Variables (Volatile Discipline)
static volatile bool     g_measure   = false;
static volatile uint32_t g_period_ms = 500;
static char              g_mode[16]  = "normal";
static esp_timer_handle_t g_timer;

// Fast hardware timer interrupt callback
static void timer_callback(void* arg) { 
    g_measure = true; 
}

// Dynamically adjust hardware periodic interval
static void restart_timer(uint32_t ms) {
    esp_timer_stop(g_timer);
    esp_timer_start_periodic(g_timer, (uint64_t)ms * 1000);
}

// Low-level register write transmission via I2C
static int mpu_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(I2C_PORT, MPU_ADDR, buf, 2, pdMS_TO_TICKS(50));
}

// Low-level burst block read transmission via I2C
static int mpu_read_bytes(uint8_t reg, uint8_t* data, int len) {
    return i2c_master_write_read_device(I2C_PORT, MPU_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(50));
}

// UART Telecommand Parser Logic
static void parse_command(char* line) {
    char cmd[32]; 
    int val;
    
    // Validate command signature: p <ms>
    if (sscanf(line, "%31s %d", cmd, &val) == 2 && strcmp(cmd, "p") == 0) {
        if (val >= 50 && val <= 5000) {
            g_period_ms = (uint32_t)val;
            restart_timer(g_period_ms);
            
            if (val <= 200)      snprintf(g_mode, sizeof(g_mode), "fast");
            else if (val >= 800) snprintf(g_mode, sizeof(g_mode), "slow");
            else                 snprintf(g_mode, sizeof(g_mode), "normal");
            
            printf("ok period=%ums mode=%s\n", (unsigned)g_period_ms, g_mode);
        } else {
            printf("err period out of range (50-5000ms)\n");
        }
        return;
    }
    
    // Validate command signature: reset
    if (strncmp(line, "reset", 5) == 0) {
        g_period_ms = 500;
        snprintf(g_mode, sizeof(g_mode), "normal");
        restart_timer(g_period_ms);
        printf("ok reset period=500ms mode=normal\n");
        return;
    }
    
    printf("err unknown command\n");
}

// Asynchronous background UART receiver task
static void uart_rx_task(void* arg) {
    uint8_t data[1];
    char line_buf[CMD_BUF_SIZE];
    int line_pos = 0;

    while (1) {
        int len = uart_read_bytes(UART_PORT, data, 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            char c = (char)data[0];
            if (c == '\r' || c == '\n') {
                if (line_pos > 0) {
                    line_buf[line_pos] = '\0';
                    parse_command(line_buf);
                    line_pos = 0;
                }
            } else if (line_pos < CMD_BUF_SIZE - 1) {
                line_buf[line_pos++] = c;
            }
        }
    }
}

// Main operational firmware entry point
extern "C" void app_main(void) {
    // 1. Initialize Peripheral UART Driver
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(UART_PORT, &uart_config);
    uart_driver_install(UART_PORT, UART_BUF_SIZE * 2, 0, 0, NULL, 0);

    // 2. Initialize Hardware I2C Master Bus Configuration
    i2c_config_t i2c_cfg = {};
    i2c_cfg.mode             = I2C_MODE_MASTER;
    i2c_cfg.sda_io_num       = (gpio_num_t)I2C_SDA;
    i2c_cfg.scl_io_num       = (gpio_num_t)I2C_SCL;
    i2c_cfg.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    i2c_cfg.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    i2c_cfg.master.clk_speed = I2C_FREQ;
    i2c_param_config(I2C_PORT, &i2c_cfg);
    i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);

    // Wake up MPU-6050 internal power domains
    vTaskDelay(pdMS_TO_TICKS(100));
    mpu_write_reg(REG_PWR_MGMT, 0x00);

    // 3. Instantiate Periodic High-Resolution Hardware Timer
    esp_timer_create_args_t timer_args = {};
    timer_args.callback = timer_callback;
    timer_args.name     = "sensor_timer";
    esp_timer_create(&timer_args, &g_timer);
    esp_timer_start_periodic(g_timer, (uint64_t)g_period_ms * 1000);

    // 4. Create Background Non-blocking UART Task
    xTaskCreate(uart_rx_task, "uart_cmd", 2048, NULL, 5, NULL);

    printf("Device Ready. Commands: p <ms> | reset\n");

    // Main telemetry streaming execution loop
    while (1) {
        if (g_measure) {
            g_measure = false; 
            uint8_t raw[6] = {0}; 
            
            // Fetch 6-axis hardware buffer bytes safely
            mpu_read_bytes(REG_ACCEL_X, raw, 6);
            
            int16_t ax16 = (int16_t)((raw[0] << 8) | raw[1]);
            int16_t ay16 = (int16_t)((raw[2] << 8) | raw[3]);
            int16_t az16 = (int16_t)((raw[4] << 8) | raw[5]);
            
            float ax = ax16 / ACCEL_SCALE;
            float ay = ay16 / ACCEL_SCALE;
            float az = az16 / ACCEL_SCALE;
            
            uint32_t t_ms = (uint32_t)(esp_timer_get_time() / 1000);
            
            // Print formatted telemetry stream over UART console
            printf("t=%u ms ax=%.2f ay=%.2f az=%.2f mode=%s\n",
                   (unsigned)t_ms, ax, ay, az, g_mode);
        }
        // Yield processor control slice to feed FreeRTOS watchdog
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

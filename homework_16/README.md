# BallisticApp Hardware Layer: MPU-6050 I2C Driver Implementation

## Overview
This module contains a native, lightweight C++20 object-oriented driver for the **MPU-6050** Inertial Measurement Unit (IMU). Communication is managed directly via the Linux native `linux/i2c-dev.h` core system interface (`open`, `ioctl`, `read`, `write`), strictly avoiding third-party external hardware abstractions or sensor libraries.

## Project Structure & Architecture

The application decouples the I2C transport framework from the sensor's internal register logic:

* **`I2cDevice` Class** (`include/BallisticApp/hardware/I2cDevice.h`, `src/I2cDevice.cpp`)
  A dedicated RAII wrapper managing the low-level Linux bus handle lifespan. It triggers `::open()` and `::ioctl(I2C_SLAVE)` within constructors and automates cleanup with `::close()` inside destructors. Features structural two-phase register reading transactions (`write(reg)` -> `read(count)` sequence). It contains zero sensor-specific logic.

* **`Mpu6050` Class** (`include/BallisticApp/hardware/Mpu6050.h`, `src/Mpu6050.cpp`)
  Encapsulates the specific operational algorithms of the MPU-6050 hardware node:
  * `checkWhoAmI()`: Queries register `0x75` to verify identity metric matching `0x68`.
  * `wake()`: Writes `0x00` to `PWR_MGMT_1` (`0x6B`) to transition the hardware out of default power-saving sleep modes.
  * `readSample()`: Executes a unified 14-byte atomic burst-read starting from `ACCEL_XOUT_H` (`0x3B`), gathering Accelerometer (6B), Temperature (2B), and Gyroscope (6B) metrics. Raw Big-Endian fields are converted to physical values using default scale presets (`+/-2g` -> 16384 LSB/g; `+/-250°/s` -> 131 LSB/dps; `T[°C] = raw/340.0 + 36.53`).

* **Main Interface CLI** (`src/main.cpp`)
  Exposes the executable runtime arguments, formats sequential metric frames via the compile-time verified `BallisticApp::Log` engine, and handles communication faults (such as missing devices, NACK responses, or invalid IDs) using exception handling blocks to return non-zero exit codes.

## Build Setup

To bypass target binary memory checking layers conflicting with virtual file preloads, generate a production-ready `Release` build context:

```bash
rm -rf build/release
cmake -B build/release -DCMAKE_BUILD_TYPE=Release
cmake --build build/release
```
*(Note: If `linux/i2c-dev.h` is missing from the environment path, the target compilation phase is automatically skipped with a warning).*

## Emulated Execution Topology

To deploy and test the telemetry streaming pipeline inside simulated workspace runtimes (such as Windows WSL or Docker Devcontainers) using the virtual peripheral hook file `libi2csim.so`:

```bash
cd build/release/homework_16
LD_PRELOAD=./libi2csim.so ./homework_16 /dev/i2c-1 0x68
```

### Argument Customization Overrides
The application tracking loops can be parameterized using explicit frequency and limit markers:
```bash
LD_PRELOAD=./libi2csim.so ./homework_16 /dev/i2c-1 0x68 --count 5 --rate 5
```

## Runtime Metrics Example

```text
[LOG] SUCCESS | MPU-6050 hardware validation verified. Executing sensor stream. [Main]
[LOG] Accel: [ 0.058, -0.107,  0.990] g | Gyro: [   0.37,  -0.02,   0.03] dps | Temp: 25.0 C [Sensor]
[LOG] Accel: [ 0.035, -0.117,  0.993] g | Gyro: [   0.29,  -0.24,   0.02] dps | Temp: 25.0 C [Sensor]
```

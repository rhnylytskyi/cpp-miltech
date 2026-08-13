#ifndef MOCK_GPIOD_C_H
#define MOCK_GPIOD_C_H

#ifdef __cplusplus
extern "C" {
#endif

// Імітація структур для компілятора
typedef struct gpiod_chip gpiod_chip;
typedef struct gpiod_line gpiod_line;

// Заглушки функцій, які використовує GpioPins.cpp
inline gpiod_chip* gpiod_chip_open_by_name(const char *name) { return (gpiod_chip*)1; }
inline gpiod_line* gpiod_chip_get_line(gpiod_chip *chip, unsigned int offset) { return (gpiod_line*)1; }
inline int gpiod_line_request_output(gpiod_line *line, const char *consumer, int default_val) { return 0; }
inline void gpiod_line_release(gpiod_line *line) {}
inline void gpiod_chip_close(gpiod_chip *chip) {}
inline int gpiod_line_set_value(gpiod_line *line, int value) { return 0; }

#ifdef __cplusplus
}
#endif

#endif // MOCK_GPIOD_C_H

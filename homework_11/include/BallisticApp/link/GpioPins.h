#pragma once
#include <string>

struct gpiod_chip;
struct gpiod_line;

// Дві лінії GPIO (через libgpiod), якими автопілот сигналізує чекеру:
//   START — піднімається в 1 одразу на старті й тримається (readiness).
//   DROP  — короткий імпульс (50-100 мс) один раз, у момент скиду боєприпасу.
// Той самий клас використовується і в симуляції (gpio-sim, --gpiochip gpiochipN),
// і на реальній платі (--gpiochip gpiochip0) — відрізняється лише ім'я чипа/номери ліній.
class GpioPins {
public:
  GpioPins() = default;
  ~GpioPins();

  GpioPins(const GpioPins &) = delete;
  GpioPins &operator=(const GpioPins &) = delete;

  // Відкрити чип, запросити START/DROP як виходи (початкове значення 0).
  // Кидає std::runtime_error при помилці.
  void open(const std::string &chipName, unsigned int startLine, unsigned int dropLine);

  void close();

  // Підняти/опустити START (готовий / не готовий).
  void setStart(bool value);

  // Короткий імпульс DROP: 1 -> sleep(durationMs) -> 0. Блокуюче, одноразове.
  void pulseDrop(int durationMs = 80);

private:
  gpiod_chip *chip_ = nullptr;
  gpiod_line *startLine_ = nullptr;
  gpiod_line *dropLine_ = nullptr;
};

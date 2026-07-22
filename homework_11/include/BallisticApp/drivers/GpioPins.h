#pragma once
#include <string>

struct gpiod_chip;
struct gpiod_line;

namespace BallisticApp::sys {

// Дві лінії GPIO (через libgpiod), якими автопілот сигналізує чекеру.
class GpioPins {
public:
  GpioPins() = default;
  ~GpioPins();

  GpioPins(const GpioPins &) = delete;
  GpioPins &operator=(const GpioPins &) = delete;

  // Відкрити чип, запросити START/DROP як виходи (початкове значення 0).
  void open(const std::string &chipName, unsigned int startLine, unsigned int dropLine);
  void close();

  // Підняти/опустити START (готовий / не готовий).
  void setStart(bool value);

  // Короткий імпульс DROP: 1 -> sleep(durationMs) -> 0. Блокуюче, одноразове.
  void pulseDrop(int durationMs = 80);

private:
  gpiod_chip *m_chip = nullptr;
  gpiod_line *m_startLine = nullptr;
  gpiod_line *m_dropLine = nullptr;
};

}  // namespace BallisticApp::sys

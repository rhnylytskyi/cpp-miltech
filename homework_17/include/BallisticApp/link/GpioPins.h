#pragma once

#include <string>

struct gpiod_chip;
struct gpiod_line;

namespace BallisticApp {

class GpioPins {
public:
  GpioPins() = default;
  ~GpioPins() noexcept;

  GpioPins(const GpioPins&) = delete;
  GpioPins& operator=(const GpioPins&) = delete;

  void open(const std::string& chipName, unsigned int startLine, unsigned int dropLine);
  void close() noexcept;

  void setStart(bool value);
  void pulseDrop(int durationMs = 80);

private:
  gpiod_chip* m_chip = nullptr;
  gpiod_line* m_startLine = nullptr;
  gpiod_line* m_dropLine = nullptr;
};

}  // namespace BallisticApp

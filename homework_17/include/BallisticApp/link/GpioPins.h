#pragma once

#include <string>

struct gpiod_chip;
struct gpiod_line;

namespace BallisticApp::link {

/**
 * @brief Hardware interaction layer with GPIO lines via libgpiod.
 */
class GpioPins {
public:
  GpioPins() = default;
  ~GpioPins() noexcept;

  /* Prevent duplication of hardware pin descriptors */
  GpioPins(const GpioPins &) = delete;
  GpioPins &operator=(const GpioPins &) = delete;

  /* Resource lifecycle and pin configuration */
  void open(const std::string &chipName, unsigned int startLine, unsigned int dropLine);
  void close() noexcept;

  /* Signal control interfaces */
  void setStart(bool value);
  void pulseDrop(int durationMs = 80);

private:
  gpiod_chip *m_chip = nullptr;
  gpiod_line *m_startLine = nullptr;
  gpiod_line *m_dropLine = nullptr;
};

}  // namespace BallisticApp::link

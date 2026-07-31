#pragma once

#include <string>

/* Forward declarations for libgpiod C-types to decouple compilation dependencies */
struct gpiod_chip;
struct gpiod_line;

namespace BallisticApp::sys {

/**
 * @brief Manages low-level hardware interaction with GPIO lines via libgpiod.
 * Controls critical handshake signals (START) and payload release mechanisms (DROP).
 */
class GpioPins {
public:
  GpioPins() = default;
  ~GpioPins() noexcept;

  /* Strict resource gating: hardware pin descriptors must not be duplicated */
  GpioPins(const GpioPins &) = delete;
  GpioPins &operator=(const GpioPins &) = delete;

  /**
   * @brief Opens the designated GPIO chip and initializes lines as output (default low).
   * @throws std::runtime_error if hardware chip initialization or layout request fails.
   */
  void open(const std::string &chipName, unsigned int startLine, unsigned int dropLine);

  /**
   * @brief Cleanly releases all requested line handles and closes the active chip context.
   */
  void close() noexcept;

  /**
   * @brief Sets the state of the START line to signal simulation readiness to the checker.
   */
  void setStart(bool value);

  /**
   * @brief Generates a blocking square pulse on the DROP line for safe weapon/pyro release.
   */
  void pulseDrop(int durationMs = 80);

private:
  gpiod_chip *m_chip = nullptr;
  gpiod_line *m_startLine = nullptr;
  gpiod_line *m_dropLine = nullptr;
};

}  // namespace BallisticApp::sys

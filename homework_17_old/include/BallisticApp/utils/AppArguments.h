#pragma once

#include <span>
#include <string>
#include <cstdint>

namespace BallisticApp {

class AppArguments {
private:
  std::string m_uart = "/tmp/ttyA";
  std::string m_gpiochip = "gpiochip0";
  unsigned int m_startLine = 24;
  unsigned int m_dropLine = 23;
  std::string m_mavlinkHost{"127.0.0.1"};
  uint16_t m_mavlinkPort{14550};

  void parse(std::span<const char* const> args);
  void validate() const;
  void printHelp() const;

public:
  explicit AppArguments(std::span<const char* const> args);
  ~AppArguments() noexcept = default;

  /* Arguments parser configuration is immutable after initialization */
  AppArguments(const AppArguments&) = default;
  AppArguments& operator=(const AppArguments&) = default;

  [[nodiscard]] const std::string& getUart() const noexcept;
  [[nodiscard]] const std::string& getGpioChip() const noexcept;
  [[nodiscard]] unsigned int getStartLine() const noexcept;
  [[nodiscard]] unsigned int getDropLine() const noexcept;
  [[nodiscard]] const std::string& getMavlinkHost() const noexcept { return m_mavlinkHost; }
  [[nodiscard]] uint16_t getMavlinkPort() const noexcept { return m_mavlinkPort; }
};

}  // namespace BallisticApp

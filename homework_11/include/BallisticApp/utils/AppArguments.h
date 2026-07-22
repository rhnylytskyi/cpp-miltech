#pragma once
#include <span>
#include <string>

namespace BallisticApp {

class AppArguments {
private:
  std::string m_uart = "/tmp/ttyA";
  std::string m_gpiochip = "gpiochip0";
  unsigned int m_startLine = 24;
  unsigned int m_dropLine = 23;

  void parse(std::span<const char* const> args);
  void validate() const;
  void printHelp() const;

public:
  explicit AppArguments(std::span<const char* const> args);

  const std::string& getUart() const;
  const std::string& getGpioChip() const;
  unsigned int getStartLine() const;
  unsigned int getDropLine() const;
};

}  // namespace BallisticApp

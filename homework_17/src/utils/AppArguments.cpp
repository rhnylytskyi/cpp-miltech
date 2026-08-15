#include "BallisticApp/utils/AppArguments.h"
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <charconv>
#include <cstdlib>

namespace BallisticApp {

AppArguments::AppArguments(std::span<const char* const> args)
{
  parse(args);
}

void AppArguments::printHelp() const
{
  std::cout << "Usage: ./drone_autopilot [options]\n\n"
            << "Options:\n"
            << "  --uart <device>         UART device serial port (Default: /tmp/ttyA)\n"
            << "  --gpiochip <name>       GPIO chip control name (Default: gpiochip0)\n"
            << "  --start-line <n>        START handshake signal line (Default: 24)\n"
            << "  --drop-line <n>         DROP weapon release line (Default: 23)\n"
            << "  --mavlink <host:port>   MAVLink UDP broadcast target endpoint (Default: 127.0.0.1:14550)\n"
            << "  -h, --help              Display help menu and exit\n";
}

void AppArguments::parse(std::span<const char* const> args)
{
  auto helpIt = std::find_if(args.begin(), args.end(), [](std::string_view arg) { return arg == "--help" || arg == "-h"; });

  if (helpIt != args.end()) {
    printHelp();
    std::exit(0);
  }

  auto getAssociatedValue = [&](auto it, std::string_view flagName) -> std::string_view {
    if (std::next(it) == args.end()) {
      throw std::runtime_error("Error: Missing value for flag: " + std::string(flagName));
    }
    return *std::next(it);
  };

  for (auto it = args.begin(); it != args.end(); ++it) {
    std::string_view arg = *it;

    if (arg == "--uart") {
      m_uart = getAssociatedValue(it, "--uart");
      ++it;
    }
    else if (arg == "--gpiochip") {
      m_gpiochip = getAssociatedValue(it, "--gpiochip");
      ++it;
    }
    else if (arg == "--start-line") {
      std::string_view val = getAssociatedValue(it, "--start-line");
      std::from_chars(val.data(), val.data() + val.size(), m_startLine);
      ++it;
    }
    else if (arg == "--drop-line") {
      std::string_view val = getAssociatedValue(it, "--drop-line");
      std::from_chars(val.data(), val.data() + val.size(), m_dropLine);
      ++it;
    }
    else if (arg == "--mavlink") {
      std::string_view hostPort = getAssociatedValue(it, "--mavlink");
      ++it;
      auto colon = hostPort.rfind(':');
      if (colon == std::string_view::npos) {
        m_mavlinkHost = std::string(hostPort);
      }
      else {
        m_mavlinkHost = std::string(hostPort.substr(0, colon));
        m_mavlinkPort = static_cast<uint16_t>(std::atoi(std::string(hostPort.substr(colon + 1)).c_str()));
      }
    }
    else if (it != args.begin()) {
      throw std::runtime_error("Error: Unknown argument: " + std::string(arg));
    }
  }

  validate();
}

void AppArguments::validate() const
{
  if (m_uart.empty()) {
    throw std::runtime_error("Error: UART device path cannot be empty!");
  }
  if (m_gpiochip.empty()) {
    throw std::runtime_error("Error: GPIO chip name cannot be empty!");
  }
  if (m_startLine == m_dropLine) {
    throw std::runtime_error("Error: START line index cannot match DROP line index.");
  }
  if (m_mavlinkHost.empty()) {
    throw std::runtime_error("Error: MAVLink UDP target host cannot be empty!");
  }
}

const std::string& AppArguments::getUart() const noexcept
{
  return m_uart;
}
const std::string& AppArguments::getGpioChip() const noexcept
{
  return m_gpiochip;
}
unsigned int AppArguments::getStartLine() const noexcept
{
  return m_startLine;
}
unsigned int AppArguments::getDropLine() const noexcept
{
  return m_dropLine;
}
const std::string& AppArguments::getMavlinkHost() const noexcept
{
  return m_mavlinkHost;
}
uint16_t AppArguments::getMavlinkPort() const noexcept
{
  return m_mavlinkPort;
}

}  // namespace BallisticApp

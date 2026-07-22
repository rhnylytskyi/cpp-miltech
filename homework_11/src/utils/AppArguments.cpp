#include "BallisticApp/utils/AppArguments.h"
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <string_view>

namespace BallisticApp {

AppArguments::AppArguments(std::span<const char* const> args)
{
  parse(args);
}

void AppArguments::printHelp() const
{
  std::cout << "===================================================================\n"
            << "  BallisticApp Drone Autopilot Board CLI (HW-in-the-Loop Mode)\n"
            << "===================================================================\n"
            << "Usage:\n"
            << "  ./ballistic_app_11 [options]\n\n"
            << "Options:\n"
            << "  --uart <device>         Specify path to the UART device serial port\n"
            << "                          (Default: /tmp/ttyA)\n"
            << "  --gpiochip <name>       Specify GPIO chip control name\n"
            << "                          (Default: gpiochip0)\n"
            << "  --start-line <n>        Specify hardware line index for START handshake signal\n"
            << "                          (Default: 24)\n"
            << "  --drop-line <n>         Specify hardware line index for DROP pyrotechnic/weapon release\n"
            << "                          (Default: 23)\n"
            << "  -h, --help              Display this autopilot configuration help menu and exit\n"
            << "===================================================================\n";
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
      throw std::runtime_error("Error: Missing associated parameter value for flag: " + std::string(flagName));
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
      m_startLine = static_cast<unsigned int>(std::stoul(std::string(val)));
      ++it;
    }
    else if (arg == "--drop-line") {
      std::string_view val = getAssociatedValue(it, "--drop-line");
      m_dropLine = static_cast<unsigned int>(std::stoul(std::string(val)));
      ++it;
    }
    else if (it != args.begin()) {
      throw std::runtime_error("Error: Unknown command line argument detected: " + std::string(arg));
    }
  }

  validate();
}

void AppArguments::validate() const
{
  if (m_uart.empty()) {
    throw std::runtime_error("Error: UART device path environment configuration cannot be empty!");
  }
  if (m_gpiochip.empty()) {
    throw std::runtime_error("Error: GPIO chip controller descriptor target name cannot be empty!");
  }
  if (m_startLine == m_dropLine) {
    throw std::runtime_error("Error: Critical IO overlap! START line index cannot match DROP line index.");
  }
}

const std::string& AppArguments::getUart() const
{
  return m_uart;
}

const std::string& AppArguments::getGpioChip() const
{
  return m_gpiochip;
}

unsigned int AppArguments::getStartLine() const
{
  return m_startLine;
}

unsigned int AppArguments::getDropLine() const
{
  return m_dropLine;
}

}  // namespace BallisticApp

#include "BallisticApp/mission/Autopilot.h"
#include "BallisticApp/drivers/GpioPins.h"
#include "BallisticApp/drivers/UartLink.h"
#include "BallisticApp/utils/AppArguments.h"
#include "BallisticApp/ComponentFactory.h"
#include "BallisticApp/utils/Logger.h"
#include "drone_link.h"
#include <span>
#include <iostream>
#include <memory>

using namespace BallisticApp;

int main(int argc, char* argv[])
{
  using UartLink = BallisticApp::sys::UartLink;
  using GpioPins = BallisticApp::sys::GpioPins;
  using Autopilot = BallisticApp::mission::Autopilot;

  try {
    std::span<const char* const> spanArgs(argv, argc);
    AppArguments appArgs(spanArgs);

    APP_LOG("{:.<15}{}", "UART_PORT", appArgs.getUart());
    APP_LOG("{:.<15}{}", "GPIO_CHIP", appArgs.getGpioChip());
    APP_LOG("{:.<15}START={},DROP={}", "GPIO_LINES", appArgs.getStartLine(), appArgs.getDropLine());

    UartLink link;
    link.open(appArgs.getUart());

    GpioPins gpio;
    gpio.open(appArgs.getGpioChip(), appArgs.getStartLine(), appArgs.getDropLine());

    dlink::AmmoCfg ammo{};
    dlink::DroneCfg cfg{};

    APP_LOG_MOD("Main", "START line raised high. Waiting for checker configuration packages...");

    // Виклик виглядає точно так само, як у вашій першій версії коду
    if (!Autopilot::handshake(link, gpio, ammo, cfg, 2000)) {
      std::cerr << "student: handshake failed — checker did not respond\n";
      return 1;
    }

    APP_LOG_MOD("Main", "SUCCESS | Handshake finalized. Loading ballistic solver...");

    auto solver = ComponentFactory::createSolver(SolverType::TABLE);
    if (!solver->initialize("data/ballistic_table.txt")) {
      throw std::runtime_error("Critical: Failed to initialize Ballistic Table Solver.");
    }

    Autopilot autopilot(link, gpio, std::move(solver), ammo, cfg);

    APP_LOG_MOD("Main", "student: flying...");

    while (autopilot.step()) {
      // Усередині step() викликається link.pump()
    }

    APP_LOG_MOD("Main", "student: done (dropped={})", autopilot.dropped() ? "yes" : "no");
  }
  catch (const std::exception& e) {
    std::cerr << "student: error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}

#include "BallisticApp/mission/Autopilot.h"
#include "BallisticApp/sys/GpioPins.h"
#include "BallisticApp/sys/UartLink.h"
#include "BallisticApp/utils/AppArguments.h"
#include "BallisticApp/ComponentFactory.h"
#include "BallisticApp/utils/Logger.h"
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <span>

using namespace BallisticApp;

int main(int argc, char* argv[])
{
  try {
    std::span<const char* const> spanArgs(argv, argc);
    AppArguments appArgs(spanArgs);

    sys::UartLink link;
    link.open(appArgs.getUart());

    sys::GpioPins gpio;
    gpio.open(appArgs.getGpioChip(), appArgs.getStartLine(), appArgs.getDropLine());

    // Silent background flush of the UART hardware buffers
    while (link.pump() > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    link.resetParser();

    dlink::AmmoCfg ammo{};
    dlink::DroneCfg cfg{};

    constexpr int kHandshakeTimeoutMs = 5000;
    if (!mission::Autopilot::handshake(link, gpio, ammo, cfg, kHandshakeTimeoutMs)) {
      std::cerr << "autopilot: handshake failed (timeout)\n";
      return 1;
    }

    auto solver = ComponentFactory::createSolver(SolverType::TABLE);
    std::string tablePath = std::string(PROJECT_DATA_DIR) + "/ballistic_table.txt";

    if (!solver->initialize(tablePath)) {
      throw std::runtime_error("Critical: Failed to initialize Ballistic Table Solver.");
    }

    mission::Autopilot autopilot(link, gpio, std::move(solver), ammo, cfg, appArgs.getMavlinkHost(), appArgs.getMavlinkPort());
    APP_LOG_MOD("Main", "autopilot: engaged, flying...");

    while (autopilot.step()) {
      // Core continuous real-time flight loop execution
    }

    APP_LOG_MOD("Main", "autopilot: finished (dropped={})", autopilot.dropped() ? "yes" : "no");
  }
  catch (const std::exception& e) {
    std::cerr << "autopilot: error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}

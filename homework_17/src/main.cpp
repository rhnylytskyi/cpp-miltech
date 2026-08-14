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

    /* Background flush to clear hardware OS input buffers before starting */
    while (link.pump() > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    link.resetParser();

    dlink::AmmoCfg ammo{};
    dlink::DroneCfg cfg{};

    /* Synchronous handshake sequence execution before spinning processing threads */
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

    mission::Autopilot autopilot(link, gpio, std::move(solver), ammo, cfg);
    APP_LOG_MOD("Main", "autopilot: multi-threaded subsystems initializing...");

    /* THREAD EXECUTION SETUP MAPPED FROM THE HOMEWORK 10 SPECIFICATIONS */
    std::thread ioThread(&mission::Autopilot::runIO, &autopilot);
    std::thread missionThread(&mission::Autopilot::runMission, &autopilot);

    /* Hardware thread sync readiness barrier gate loop */
    while (!autopilot.isThreadReady()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    /* Simultaneous background task execution trigger */
    autopilot.start();
    APP_LOG_MOD("Main", "autopilot: engaged multi-threading real-time loop!");

    /* Polling reactor loop that awaits official simulation finalization packet */
    while (!autopilot.isFinished()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    /* Cleanly stop and join all internal subsystem thread allocations */
    autopilot.stop();

    if (missionThread.joinable()) {
      missionThread.join();
    }
    if (ioThread.joinable()) {
      ioThread.join();
    }

    APP_LOG_MOD("Main", "autopilot: finished (dropped={})", autopilot.dropped() ? "yes" : "no");
  }
  catch (const std::exception& e) {
    std::cerr << "autopilot: error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}

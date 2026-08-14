#include "BallisticApp/link/Autopilot.h"
#include "BallisticApp/link/GpioPins.h"
#include "BallisticApp/link/UartLink.h"
#include "BallisticApp/net/MavlinkTelemetry.h"
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

    link::UartLink link;
    link.open(appArgs.getUart());

    link::GpioPins gpio;
    gpio.open(appArgs.getGpioChip(), appArgs.getStartLine(), appArgs.getDropLine());

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

    // Allocate and trigger network thread loops before configuring the mission flight computer
    auto mavlink = std::make_shared<net::MavlinkTelemetry>(appArgs.getMavlinkHost(), appArgs.getMavlinkPort());
    mavlink->start();
    std::cout << "[Main] MAVLink network gateway active -> " << appArgs.getMavlinkHost() << ":" << appArgs.getMavlinkPort() << "\n";

    // Build multi-threaded autopilot and inject shared network handler reference
    mission::Autopilot autopilot(link, gpio, std::move(solver), ammo, cfg, mavlink);
    APP_LOG_MOD("Main", "autopilot: multi-threaded subsystems initializing...");

    std::thread ioThread(&mission::Autopilot::runIO, &autopilot);
    std::thread missionThread(&mission::Autopilot::runMission, &autopilot);

    while (!autopilot.isThreadReady()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    autopilot.start();
    APP_LOG_MOD("Main", "autopilot: engaged multi-threading real-time loop!");

    if (missionThread.joinable()) {
      missionThread.join();
    }

    autopilot.stop();

    if (ioThread.joinable()) {
      ioThread.join();
    }

    // Safely spin down and detach network socket contexts post flight execution
    mavlink->stop();
    APP_LOG_MOD("Main", "autopilot: finished (dropped={})", autopilot.dropped() ? "yes" : "no");
  }
  catch (const std::exception& e) {
    std::cerr << "autopilot: error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}

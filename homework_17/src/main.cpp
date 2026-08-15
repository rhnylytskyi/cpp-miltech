#include "BallisticApp/ComponentFactory.h"
#include "BallisticApp/exporters/JsonExporter.h"
#include "BallisticApp/link/GpioPins.h"
#include "BallisticApp/link/UartLink.h"
#include "BallisticApp/mission/Autopilot.h"
#include "BallisticApp/net/MavlinkTelemetry.h"
#include "BallisticApp/utils/AppArguments.h"
#include "BallisticApp/utils/Logger.h"
#include <chrono>
#include <iostream>
#include <memory>
#include <span>
#include <thread>

using namespace BallisticApp;

int main(int argc, char* argv[])
{
  try {
    std::span<const char* const> spanArgs(argv, argc);
    AppArguments appArgs(spanArgs);

    UartLink link;
    link.open(appArgs.getUart());

    GpioPins gpio;
    gpio.open(appArgs.getGpioChip(), appArgs.getStartLine(), appArgs.getDropLine());

    while (link.pump() > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    link.resetParser();

    dlink::AmmoCfg ammo{};
    dlink::DroneCfg cfg{};

    constexpr int kHandshakeTimeoutMs = 5000;
    if (!Autopilot::handshake(link, gpio, ammo, cfg, kHandshakeTimeoutMs)) {
      std::cerr << "autopilot: handshake failed (timeout)\n";
      return 1;
    }

    auto solver = ComponentFactory::createSolver(SolverType::TABLE);
    std::string tablePath = std::string(PROJECT_DATA_DIR) + "/ballistic_table.txt";

    if (!solver->initialize(tablePath)) {
      throw std::runtime_error("Critical: Failed to initialize Ballistic Table Solver.");
    }

    auto mavlink = std::make_shared<MavlinkTelemetry>(appArgs.getMavlinkHost(), appArgs.getMavlinkPort());
    mavlink->start();
    APP_LOG_MOD("Main", "autopilot: MAVLink network gateway active -> {}:{}", appArgs.getMavlinkHost(), appArgs.getMavlinkPort());

    Autopilot autopilot(link, gpio, std::move(solver), ammo, cfg, mavlink);
    APP_LOG_MOD("Main", "autopilot: multi-threaded subsystems initializing...");

    std::thread ioThread(&Autopilot::runIO, &autopilot);
    std::thread missionThread(&Autopilot::runMission, &autopilot);

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

    mavlink->stop();
    APP_LOG_MOD("Main", "autopilot: core execution finished cleanly (dropped={})", autopilot.dropped() ? "yes" : "no");

    JsonExporter exporter;
    for (const auto& step : autopilot.getSimulationSteps()) {
      exporter.record(step);
    }

    std::string outputPath = std::string(PROJECT_DATA_DIR) + "/simulation.json";
    if (exporter.save(outputPath)) {
      APP_LOG_MOD("Main", "autopilot: mission track profile saved -> simulation.json");
    }
  }
  catch (const std::exception& e) {
    std::cerr << "autopilot: error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}

#include "BallisticApp/mission/MissionProcessor.h"
#include "BallisticApp/mission/AppArguments.h"
#include "BallisticApp/navigation/DronePhysics.h"
#include "BallisticApp/providers/ThreadSafeTargetProvider.h"
#include "BallisticApp/utils/Logger.h"
#include <iostream>
#include <exception>
#include <thread>
#include <chrono>
#include <span>

using namespace BallisticApp;

int main(int argc, char* argv[])
{
  try {
    std::span<const char* const> spanArgs(argv, argc);
    AppArguments appArgs(spanArgs);

    APP_LOG("{:.<15} {}", "CONFIG_FILE", appArgs.getConfigPath().string());
    APP_LOG("{:.<15} {}", "TARGET_LIST", appArgs.getTargetsPath().string());
    APP_LOG("{:.<15} {}", "SOLVER_TYPE", appArgs.getSolverType() == SolverType::TABLE ? "TABLE" : "ANALYTICAL");

    ThreadSafeTargetProvider provider(appArgs.getTargetsPath());
    if (!provider.load()) {
      std::cerr << "Error: Failed to load target trajectories." << std::endl;
      return 1;
    }

    DronePhysics physics;
    MissionProcessor mission(appArgs, provider, physics);

    std::thread providerThread(&ThreadSafeTargetProvider::run, &provider);
    std::thread physicsThread(&DronePhysics::run, &physics);
    std::thread missionThread(&MissionProcessor::run, &mission);

    while (!provider.isThreadReady() || !physics.isThreadReady() || !mission.isThreadReady()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    provider.start();
    physics.start();
    mission.start();

    if (missionThread.joinable()) {
      missionThread.join();
    }

    physics.stop();
    provider.stop();

    if (physicsThread.joinable()) {
      physicsThread.join();
    }
    if (providerThread.joinable()) {
      providerThread.join();
    }

    const auto history = mission.getStepsHistory();
    if (!history.empty()) {
      float totalSimTime = history.back().timeSecSinceStart;
      APP_LOG_MOD("Main", "SUCCESS | Simulation finalized. Total Steps: {} | Flight Time: {:.2f}s", history.size(), totalSimTime);
    }
    else {
      APP_LOG_MOD("Main", "WARNING | Simulation completed with empty history.");
    }
  }
  catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}

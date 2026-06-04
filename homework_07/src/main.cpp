#include "BallisticApp/mission/MissionProcessor.h"
#include "BallisticApp/mission/AppArguments.h"
#include "BallisticApp/utils/Logger.h"
#include <iostream>
#include <exception>

using namespace BallisticApp;

int main(int argc, char* argv[])
{
  try {
    std::span<const char* const> spanArgs(argv, argc);
    AppArguments appArgs(spanArgs);

    APP_LOG("{:.<15} {}", "CONFIG_FILE", appArgs.getConfigPath().string());
    APP_LOG("{:.<15} {}", "TARGET_LIST", appArgs.getTargetsPath().string());
    APP_LOG("{:.<15} {}", "SOLVER_TYPE", appArgs.getSolverType() == SolverType::TABLE ? "TABLE" : "ANALYTICAL");

    LaunchParams params{.configPath = appArgs.getConfigPath(),
                        .targetsPath = appArgs.getTargetsPath(),
                        .ammoPath = appArgs.getAmmoPath(),
                        .simulationPath = appArgs.getSimulationPath(),
                        .tablePath = appArgs.getTablePath(),
                        .solverType = appArgs.getSolverType(),
                        .enableTargetLock = appArgs.isTargetLockEnabled()};

    MissionProcessor mission(params);
    mission.run();
  }
  catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}

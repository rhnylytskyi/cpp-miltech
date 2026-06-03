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

    APP_LOG("Config loaded from: {}", appArgs.getConfigPath().string());
    APP_LOG("Targets loaded from: {}", appArgs.getTargetsPath().string());

    MissionProcessor mission(
      appArgs.getConfigPath(), appArgs.getTargetsPath(), appArgs.getAmmoPath(), appArgs.getSimulationPath(), appArgs.isTargetLockEnabled());

    mission.run();
  }
  catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}

#include "BallisticApp/core/MissionProcessor.h"
#include "BallisticApp/core/ScenarioPathResolver.h"
#include "BallisticApp/utils/Logger.h"
#include <iostream>
#include <exception>

using namespace BallisticApp;

int main(int argc, char* argv[])
{
  try {
    ScenarioPathResolver pathResolver({argv, static_cast<size_t>(argc)});

    APP_LOG("Loading config from: {}", pathResolver.getConfigPath().string());
    APP_LOG("Loading targets from: {}", pathResolver.getTargetsPath().string());

    MissionProcessor mission(
      pathResolver.getConfigPath(), pathResolver.getTargetsPath(), pathResolver.getAmmoPath(), pathResolver.getSimulationPath());
    mission.run();
  }
  catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}

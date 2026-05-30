#include "BallisticApp/MissionProcessor.h"
#include "BallisticApp/utils/ConfigManager.h"
#include "BallisticApp/Defines.h"
#include <iostream>

using namespace BallisticApp;

int main(int argc, char* argv[])
{
  try {
    ConfigManager configManager;
    configManager.initialize(argc, argv);

    LOG("Loading targets from: " << configManager.getTargetsPath());
    LOG("Loading config from: " << configManager.getConfigPath());

    MissionProcessor mission(
      configManager.getTargetsPath(), configManager.getSimulationPath(), configManager.getConfigPath(), configManager.getAmmoPath());

    mission.run();
  }
  catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}

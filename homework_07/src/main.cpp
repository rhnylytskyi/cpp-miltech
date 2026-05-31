#include "BallisticApp/MissionProcessor.h"
#include "BallisticApp/ScenarioPathResolver.h"
#include "BallisticApp/Defines.h"
#include <iostream>
#include <exception>

using namespace BallisticApp;

int main(int argc, char* argv[])
{
  try {
    ScenarioPathResolver pathResolver({argv, static_cast<size_t>(argc)});

    LOG("Loading config from: " << pathResolver.getConfigPath());
    LOG("Loading targets from: " << pathResolver.getTargetsPath());

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

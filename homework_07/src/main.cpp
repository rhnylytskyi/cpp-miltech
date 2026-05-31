#include "BallisticApp/MissionProcessor.h"
#include "BallisticApp/ConfigLocator.h"
#include "BallisticApp/Defines.h"
#include <iostream>
#include <exception>

using namespace BallisticApp;

int main(int argc, char* argv[])
{
  try {
    const ConfigLocator locator(argc, argv);

    LOG("Loading config from: " << locator.getConfigPath());
    LOG("Loading targets from: " << locator.getTargetsPath());

    MissionProcessor mission(locator.getConfigPath(), locator.getTargetsPath(), locator.getAmmoPath(), locator.getSimulationPath());
    mission.run();
  }
  catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}

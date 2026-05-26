#include "ballistic_app/MissionProcessor.h"
#include "ballistic_app/utils/PathResolver.h"
#include "ballistic_app/Defines.h"
#include <iostream>
#include <string>
#include <filesystem>

namespace fs = std::filesystem;
using namespace BallisticApp;

int main(int argc, char* argv[])
{
  try {
    PathResolver::parseArguments(argc, argv);

    std::string targetsPath = PathResolver::getTargetsPath();
    std::string configPath = PathResolver::getConfigPath();
    std::string ammoPath = PathResolver::getAmmoPath();
    std::string simulationPath = PathResolver::getSimulationPath();

    if (!fs::exists(configPath) || !fs::exists(targetsPath)) {
      std::cerr << "Error: Required configuration files not found!" << std::endl;
      std::cerr << "Checked paths:\n - " << configPath << "\n - " << targetsPath << std::endl;
      return 1;
    }

    LOG("Loading targets from: " << targetsPath << std::endl);
    LOG("Loading config from: " << configPath << std::endl);

    MissionProcessor mission(targetsPath, simulationPath, configPath, ammoPath);
    mission.run();
  }
  catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}

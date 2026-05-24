#include "ballistic_app/interfaces/IBallisticSolver.h"
#include "ballistic_app/interfaces/ITargetProvider.h"
#include "ballistic_app/interfaces/IConfigLoader.h"
#include "ballistic_app/interfaces/ISimulationExporter.h"
#include "ballistic_app/Defines.h"
#include "ballistic_app/config/ComponentFactory.h"
#include "ballistic_app/utils/PathResolver.h"
#include "ballistic_app/MissionProcessor.h"

#include <cstring>
#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace BallisticApp;

int main(int argc, char* argv[])
{
  try {
    PathResolver pathResolver(argc, argv);

    const char* targetsPath = pathResolver.getTargetsPath();
    const char* configPath = pathResolver.getConfigPath();

    LOG("Loading targets from: " << targetsPath << std::endl);
    LOG("Loading config from: " << configPath << std::endl);

    IConfigLoader* loader = ComponentFactory::createLoader(ConfigLoaderType::FILE);
    ITargetProvider* provider = ComponentFactory::createProvider(TargetProviderType::JSON, targetsPath);
    IBallisticSolver* solver = ComponentFactory::createSolver(SolverType::ANALYTICAL);
    ISimulationExporter* exporter = ComponentFactory::createExporter(ExporterType::JSON, SIMULATION_PATH);

    MissionProcessor mission(loader, provider, solver, exporter);
    mission.init(configPath, AMMO_PATH);
    mission.run();
  }
  catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
  }

  return 0;
}

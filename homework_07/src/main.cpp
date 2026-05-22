#include "ballistic_app/solvers/IBallisticSolver.hpp"
#include "ballistic_app/providers/ITargetProvider.hpp"
#include "ballistic_app/io/interfaces/IConfigLoader.hpp"
#include "ballistic_app/io/interfaces/ISimulationExporter.hpp"
#include "ballistic_app/Defines.hpp"
#include "ballistic_app/Factory.hpp"
#include "ballistic_app/utils/PathResolver.hpp"
#include "ballistic_app/MissionProcessor.hpp"
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

    IConfigLoader* loader = Factory::createLoader(ConfigLoaderType::FILE);
    ITargetProvider* provider = Factory::createProvider(TargetProviderType::JSON, targetsPath);
    IBallisticSolver* solver = Factory::createSolver(SolverType::ANALYTICAL);
    ISimulationExporter* exporter = Factory::createExporter(ExporterType::JSON, SIMULATION_PATH);

    MissionProcessor mission(loader, provider, solver, exporter);
    mission.init(configPath, AMMO_PATH);
    mission.run();
  }
  catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
  }

  return 0;
}

#include "BallisticApp/mission/ComponentFactory.h"
#include "BallisticApp/loaders/FileConfigLoader.h"
#include "BallisticApp/providers/JsonTargetProvider.h"
#include "BallisticApp/solvers/AnalyticalBallisticSolver.h"
#include "BallisticApp/exporters/JsonExporter.h"

namespace BallisticApp {

std::unique_ptr<IBallisticSolver> ComponentFactory::createSolver(SolverType type)
{
  return (type == SolverType::ANALYTICAL) ? std::make_unique<AnalyticalBallisticSolver>() : nullptr;
}

std::unique_ptr<ITargetProvider> ComponentFactory::createProvider(TargetProviderType type, const std::filesystem::path& param)
{
  return (type == TargetProviderType::JSON) ? std::make_unique<JsonTargetProvider>(param) : nullptr;
}

std::unique_ptr<IConfigLoader> ComponentFactory::createLoader(ConfigLoaderType type)
{
  return (type == ConfigLoaderType::FILE) ? std::make_unique<FileConfigLoader>() : nullptr;
}

std::unique_ptr<ISimulationExporter> ComponentFactory::createExporter(ExporterType type, const std::filesystem::path& param)
{
  return (type == ExporterType::JSON) ? std::make_unique<JsonExporter>(param) : nullptr;
}

}  // namespace BallisticApp

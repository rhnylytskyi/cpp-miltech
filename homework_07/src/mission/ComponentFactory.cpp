#include "BallisticApp/mission/ComponentFactory.h"
#include "BallisticApp/loaders/FileConfigLoader.h"
#include "BallisticApp/providers/JsonTargetProvider.h"
#include "BallisticApp/solvers/AnalyticalSolver.h"
#include "BallisticApp/solvers/TableSolver.h"
#include "BallisticApp/exporters/JsonExporter.h"

namespace BallisticApp {

std::unique_ptr<IBallisticSolver> ComponentFactory::createSolver(SolverType type)
{
  if (type == SolverType::ANALYTICAL) {
    return std::make_unique<AnalyticalSolver>();
  }
  if (type == SolverType::TABLE) {
    return std::make_unique<TableSolver>();
  }
  return nullptr;
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

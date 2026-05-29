#include "ballistic_app/config/ComponentFactory.h"
#include "ballistic_app/config/FileConfigLoader.h"
#include "ballistic_app/providers/JsonTargetProvider.h"
#include "ballistic_app/solvers/AnalyticalBallisticSolver.h"
#include "ballistic_app/exporters/JsonExporter.h"
#include <memory>

namespace BallisticApp {

std::unique_ptr<IBallisticSolver> ComponentFactory::createSolver(SolverType type)
{
  switch (type) {
    case SolverType::ANALYTICAL:
      return std::make_unique<AnalyticalBallisticSolver>();
    default:
      return nullptr;
  }
}

std::unique_ptr<ITargetProvider> ComponentFactory::createProvider(TargetProviderType type, const std::string& param)
{
  switch (type) {
    case TargetProviderType::JSON:
      return std::make_unique<JsonTargetProvider>(param);
    default:
      return nullptr;
  }
}

std::unique_ptr<IConfigLoader> ComponentFactory::createLoader(ConfigLoaderType type)
{
  switch (type) {
    case ConfigLoaderType::FILE:
      return std::make_unique<FileConfigLoader>();
    default:
      return nullptr;
  }
}

std::unique_ptr<ISimulationExporter> ComponentFactory::createExporter(ExporterType type, const std::string& param)
{
  switch (type) {
    case ExporterType::JSON:
      return std::make_unique<JsonExporter>(param);
    default:
      return nullptr;
  }
}

}  // namespace BallisticApp

#include "ballistic_app/config/ComponentFactory.h"
#include "ballistic_app/config/FileConfigLoader.h"
#include "ballistic_app/providers/JsonTargetProvider.h"
#include "ballistic_app/solvers/AnalyticalBallisticSolver.h"
#include "ballistic_app/exporters/JsonExporter.h"

namespace BallisticApp {

IBallisticSolver* ComponentFactory::createSolver(SolverType type)
{
  switch (type) {
    case SolverType::ANALYTICAL:
      return new AnalyticalBallisticSolver();
    default:
      return nullptr;
  }
}

ITargetProvider* ComponentFactory::createProvider(TargetProviderType type, const char* param)
{
  switch (type) {
    case TargetProviderType::JSON:
      return new JsonTargetProvider(param);
    default:
      return nullptr;
  }
}

IConfigLoader* ComponentFactory::createLoader(ConfigLoaderType type)
{
  switch (type) {
    case ConfigLoaderType::FILE:
      return new FileConfigLoader();
    default:
      return nullptr;
  }
}

ISimulationExporter* ComponentFactory::createExporter(ExporterType type, const char* param)
{
  switch (type) {
    case ExporterType::JSON:
      return new JsonExporter(param);
    default:
      return nullptr;
  }
}

}  // namespace BallisticApp

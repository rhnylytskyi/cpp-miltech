#include "ballistic_app/Factory.hpp"
#include "ballistic_app/solvers/AnalyticalBallisticSolver.hpp"
#include "ballistic_app/providers/JsonTargetProvider.hpp"
#include "ballistic_app/io/FileConfigLoader.hpp"
#include "ballistic_app/io/JsonExporter.hpp"

namespace BallisticApp {

IBallisticSolver* Factory::createSolver(SolverType type)
{
  switch (type) {
    case SolverType::ANALYTICAL:
      return new AnalyticalBallisticSolver();
    default:
      return nullptr;
  }
}

ITargetProvider* Factory::createProvider(TargetProviderType type, const char* param)
{
  switch (type) {
    case TargetProviderType::JSON:
      return new JsonTargetProvider(param);
    default:
      return nullptr;
  }
}

IConfigLoader* Factory::createLoader(ConfigLoaderType type)
{
  switch (type) {
    case ConfigLoaderType::FILE:
      return new FileConfigLoader();
    default:
      return nullptr;
  }
}

ISimulationExporter* Factory::createExporter(ExporterType type, const char* param)
{
  switch (type) {
    case ExporterType::JSON:
      return new JsonExporter(param);
    default:
      return nullptr;
  }
}

}  // namespace BallisticApp

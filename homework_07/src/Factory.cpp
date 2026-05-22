#include "ballistic_app/Factory.hpp"
#include "ballistic_app/ballistic/AnalyticalBallisticSolver.hpp"
#include "ballistic_app/target/JsonTargetProvider.hpp"
#include "ballistic_app/config/FileConfigLoader.hpp"

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

}  // namespace BallisticApp

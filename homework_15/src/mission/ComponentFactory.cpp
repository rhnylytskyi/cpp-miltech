#include "BallisticApp/mission/ComponentFactory.h"
#include "BallisticApp/loaders/FileConfigLoader.h"
#include "BallisticApp/solvers/AnalyticalSolver.h"
#include "BallisticApp/solvers/TableSolver.h"
#include "BallisticApp/exporters/JsonExporter.h"
#include <stdexcept>

namespace BallisticApp {

std::unique_ptr<IBallisticSolver> ComponentFactory::createSolver(SolverType type)
{
  switch (type) {
    case SolverType::ANALYTICAL:
      return std::make_unique<AnalyticalSolver>();
    case SolverType::TABLE:
      return std::make_unique<TableSolver>();
    default:
      throw std::invalid_argument("ComponentFactory: Unsupported or unknown SolverType.");
  }
}

std::unique_ptr<IConfigLoader> ComponentFactory::createLoader(ConfigLoaderType type)
{
  switch (type) {
    case ConfigLoaderType::FILE:
      return std::make_unique<FileConfigLoader>();
    default:
      throw std::invalid_argument("ComponentFactory: Unsupported or unknown ConfigLoaderType.");
  }
}

std::unique_ptr<ISimulationExporter> ComponentFactory::createExporter(ExporterType type, const std::filesystem::path& param)
{
  switch (type) {
    case ExporterType::JSON:
      return std::make_unique<JsonExporter>(param);
    default:
      throw std::invalid_argument("ComponentFactory: Unsupported or unknown ExporterType.");
  }
}

}  // namespace BallisticApp

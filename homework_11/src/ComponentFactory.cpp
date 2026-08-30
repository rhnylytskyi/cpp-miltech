#include "BallisticApp/ComponentFactory.h"
#include "BallisticApp/solvers/AnalyticalSolver.h"
#include "BallisticApp/solvers/TableSolver.h"
#include <stdexcept>

namespace BallisticApp {

std::unique_ptr<IBallisticSolver> ComponentFactory::createSolver(SolverType type)
{
  switch (type) {
    case SolverType::ANALYTICAL:
      return std::make_unique<solvers::AnalyticalSolver>();
    case SolverType::TABLE:
      return std::make_unique<solvers::TableSolver>();
    default:
      throw std::invalid_argument("ComponentFactory: Unsupported or unknown SolverType.");
  }
}

}  // namespace BallisticApp

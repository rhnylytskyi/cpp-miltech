#pragma once
#include "BallisticApp/interfaces/IBallisticSolver.h"
#include <memory>

namespace BallisticApp {

enum class SolverType { ANALYTICAL, TABLE };

class ComponentFactory {
public:
  static std::unique_ptr<IBallisticSolver> createSolver(SolverType type);
};

}  // namespace BallisticApp

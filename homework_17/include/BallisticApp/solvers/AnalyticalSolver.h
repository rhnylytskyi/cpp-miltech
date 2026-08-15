#pragma once

#include "BallisticApp/interfaces/IBallisticSolver.h"

namespace BallisticApp {

class AnalyticalSolver : public IBallisticSolver {
public:
  AnalyticalSolver() = default;
  ~AnalyticalSolver() override = default;

  Result calculate(float altitude, float speed, const AmmoParams& ammo) override;
};

}  // namespace BallisticApp

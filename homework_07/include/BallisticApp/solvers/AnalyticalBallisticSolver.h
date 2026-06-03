#pragma once

#include "BallisticApp/config/AmmoParams.h"
#include "BallisticApp/interfaces/IBallisticSolver.h"

namespace BallisticApp {
class AnalyticalBallisticSolver : public IBallisticSolver {
public:
  AnalyticalBallisticSolver() = default;
  ~AnalyticalBallisticSolver() override = default;

  float calcTimeOfFall(float z0, float v0, const AmmoParams& ammo) override;
  float calcHDistance(float t, float V0, const AmmoParams& ammo) override;
};

}  // namespace BallisticApp

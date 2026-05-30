#pragma once

#include "BallisticApp/AmmoParams.h"
#include "BallisticApp/interfaces/IBallisticSolver.h"

namespace BallisticApp {
class AnalyticalBallisticSolver : public IBallisticSolver {
private:
  const float g_gravity = 9.81f;

public:
  AnalyticalBallisticSolver() = default;
  ~AnalyticalBallisticSolver() override = default;

  float calcTimeOfFall(float z0, float v0, const AmmoParams& ammo) override;
  float calcHDistance(float t, float V0, const AmmoParams& ammo) override;
};

}  // namespace BallisticApp

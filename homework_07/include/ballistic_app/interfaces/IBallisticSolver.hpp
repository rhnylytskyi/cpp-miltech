#pragma once
#include "ballistic_app/dto/AmmoParams.hpp"

namespace BallisticApp {
class IBallisticSolver {
public:
  virtual ~IBallisticSolver() = default;
  virtual float calcTimeOfFall(float z0, float v0, const AmmoParams& ammo) = 0;
  virtual float calcHDistance(float t, float V0, const AmmoParams& ammo) = 0;
};
}  // namespace BallisticApp

#include "ballistic_app/ballistic/AnalyticalBallisticSolver.hpp"
#include <cmath>

namespace BallisticApp {

float AnalyticalBallisticSolver::calcTimeOfFall(float z0, float v0, const AmmoParams& ammo)
{
  float a = ammo.drag * g_gravity * ammo.mass - 2 * ammo.drag * ammo.lift * v0;
  float b = -3 * g_gravity * ammo.mass + 3 * ammo.drag * ammo.lift * v0;
  float c = 6 * ammo.mass * z0;

  if (std::fabs(a) < 1e-12f) {
    return std::sqrt(2.0f * z0 / g_gravity);
  }

  float p = -b * b / (3 * a * a);
  float q = (2 * b * b * b) / (27 * a * a * a) + c / a;

  if (p >= 0) {
    return std::sqrt(2.0f * z0 / g_gravity);
  }

  float arg = 3 * q / (2 * p) * std::sqrt(-3 / p);
  if (std::fabs(arg) > 1) {
    return std::sqrt(2.0f * z0 / g_gravity);
  }

  float phi = std::acos(arg);
  return 2 * std::sqrt(-p / 3) * std::cos((phi + 4 * (float)M_PI) / 3) - b / (3 * a);
}

float AnalyticalBallisticSolver::calcHDistance(float t, float V0, const AmmoParams& ammo)
{
  float l2 = ammo.lift * ammo.lift;
  return t * V0 - (ammo.drag * std::pow(t, 2) * V0) / (2 * ammo.mass) +
         (std::pow(t, 3) * (6 * ammo.drag * g_gravity * ammo.lift * ammo.mass - 8 * std::pow(ammo.drag, 2) * (-1 + l2) * V0)) /
           (36 * std::pow(ammo.mass, 2));
}

}  // namespace BallisticApp

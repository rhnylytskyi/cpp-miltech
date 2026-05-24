#include "ballistic_app/solvers/AnalyticalBallisticSolver.h"
#include <cmath>

namespace BallisticApp {

// ------------------------------------------------------------
// Балістика з ДЗ 1: час польоту (метод Кардано)
// ------------------------------------------------------------
float AnalyticalBallisticSolver::calcTimeOfFall(float z0, float v0, const AmmoParams& ammo)
{
  float m = ammo.mass;
  float d = ammo.drag;
  float l = ammo.lift;

  float a = d * g_gravity * m - 2.0f * d * d * l * v0;
  float b = -3.0f * g_gravity * m * m + 3.0f * d * l * m * v0;
  float c = 6.0f * m * m * z0;

  if (std::fabs(a) < 1e-12f)
    return std::sqrt(2.0f * z0 / g_gravity);

  float p = -b * b / (3.0f * a * a);
  float q = (2.0f * b * b * b) / (27.0f * a * a * a) + c / a;

  if (p >= 0)
    return std::sqrt(2.0f * z0 / g_gravity);

  float arg = 3.0f * q / (2.0f * p) * std::sqrt(-3.0f / p);
  if (std::fabs(arg) > 1.0f)
    return std::sqrt(2.0f * z0 / g_gravity);

  float phi = std::acos(arg);
  float t = 2.0f * std::sqrt(-p / 3.0f) * std::cos((phi + 4.0f * (float)M_PI) / 3.0f) - b / (3.0f * a);
  return t > 0.0f ? t : std::sqrt(2.0f * z0 / g_gravity);
}

// ------------------------------------------------------------
// Балістика з ДЗ 1: горизонтальна дистанція (степеневий ряд до t^5)
// ------------------------------------------------------------
float AnalyticalBallisticSolver::calcHDistance(float t, float V0, const AmmoParams& ammo)
{
  float m = ammo.mass;
  float d = ammo.drag;
  float l = ammo.lift;

  float l2 = l * l;
  float l4 = l2 * l2;

  // Кешування степенів часу для швидших розрахунків
  float t2 = t * t;
  float t3 = t2 * t;
  float t4 = t3 * t;
  float t5 = t4 * t;

  float d2 = d * d;
  float d3 = d2 * d;
  float d4 = d3 * d;

  float h =
    V0 * t - (d * t2 * V0) / (2.0f * m) + (t3 * (6.0f * d * g_gravity * l * m - 6.0f * d2 * (-1.0f + l2) * V0)) / (36.0f * m * m) +
    (t4 * (-6.0f * d2 * g_gravity * l * (1.0f + l2 + l4) * m + 3.0f * d3 * l2 * (1.0f + l2) * V0 + 6.0f * d3 * l4 * (1.0f + l2) * V0)) /
      (36.0f * std::pow(1.0f + l2, 2) * std::pow(m, 3)) +
    (t5 * (3.0f * d3 * g_gravity * l2 * l * m - 3.0f * d4 * l2 * (1.0f + l2) * V0)) / (36.0f * (1.0f + l2) * std::pow(m, 4));

  return h;
}

}  // namespace BallisticApp

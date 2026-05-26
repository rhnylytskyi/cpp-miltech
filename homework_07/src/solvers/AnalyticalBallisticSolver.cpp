#include "ballistic_app/solvers/AnalyticalBallisticSolver.h"
#include <cmath>
#include <numbers>

namespace BallisticApp {

// ------------------------------------------------------------
// Балістика з ДЗ 1: час польоту (метод Кардано)
// ------------------------------------------------------------
float AnalyticalBallisticSolver::calcTimeOfFall(float z0, float v0, const AmmoParams& ammo)
{
  const float m = ammo.mass;
  const float d = ammo.drag;
  const float l = ammo.lift;

  const float a = d * g_gravity * m - 2.0f * d * d * l * v0;
  const float b = -3.0f * g_gravity * m * m + 3.0f * d * l * m * v0;
  const float c = 6.0f * m * m * z0;

  if (std::fabs(a) < std::numeric_limits<float>::epsilon()) {
    return std::sqrt(2.0f * z0 / g_gravity);
  }

  const float p = -b * b / (3.0f * a * a);
  const float q = (2.0f * b * b * b) / (27.0f * a * a * a) + c / a;

  if (p >= 0.0f) {
    return std::sqrt(2.0f * z0 / g_gravity);
  }

  const float arg = 3.0f * q / (2.0f * p) * std::sqrt(-3.0f / p);
  if (std::fabs(arg) > 1.0f) {
    return std::sqrt(2.0f * z0 / g_gravity);
  }

  const float phi = std::acos(arg);

  const float t = 2.0f * std::sqrt(-p / 3.0f) * std::cos((phi + 4.0f * std::numbers::pi_v<float>) / 3.0f) - b / (3.0f * a);

  return t > 0.0f ? t : std::sqrt(2.0f * z0 / g_gravity);
}

// ------------------------------------------------------------
// Балістика з ДЗ 1: горизонтальна дистанція (степеневий ряд до t^5)
// ------------------------------------------------------------
float AnalyticalBallisticSolver::calcHDistance(float t, float V0, const AmmoParams& ammo)
{
  const float m = ammo.mass;
  const float d = ammo.drag;
  const float l = ammo.lift;

  const float l2 = l * l;
  const float l4 = l2 * l2;

  // Оптимальне обчислення степенів часу без використання важкої функції std::pow
  const float t2 = t * t;
  const float t3 = t2 * t;
  const float t4 = t3 * t;
  const float t5 = t4 * t;

  const float d2 = d * d;
  const float d3 = d2 * d;
  const float d4 = d3 * d;

  // Оптимізація: std::pow(m, 3) замінено на чисте множення m * m * m (для float це значно швидше)
  const float m2 = m * m;
  const float m3 = m2 * m;
  const float m4 = m3 * m;

  // Знаменник для третього доданку (1.0f + l2)^2
  const float denomFactor = (1.0f + l2) * (1.0f + l2);

  const float h =
    V0 * t - (d * t2 * V0) / (2.0f * m) + (t3 * (6.0f * d * g_gravity * l * m - 6.0f * d2 * (-1.0f + l2) * V0)) / (36.0f * m2) +
    (t4 * (-6.0f * d2 * g_gravity * l * (1.0f + l2 + l4) * m + 3.0f * d3 * l2 * (1.0f + l2) * V0 + 6.0f * d3 * l4 * (1.0f + l2) * V0)) /
      (36.0f * denomFactor * m3) +
    (t5 * (3.0f * d3 * g_gravity * l2 * l * m - 3.0f * d4 * l2 * (1.0f + l2) * V0)) / (36.0f * (1.0f + l2) * m4);

  return h;
}

}  // namespace BallisticApp

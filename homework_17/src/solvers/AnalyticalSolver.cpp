#include "BallisticApp/solvers/AnalyticalSolver.h"
#include "BallisticApp/config/AmmoParams.h"
#include <cmath>
#include <limits>
#include <numbers>

namespace BallisticApp::solvers {

namespace {
constexpr float g_gravity = 9.81f;
}

IBallisticSolver::Result AnalyticalSolver::calculate(float altitude, float speed, const AmmoParams& ammo)
{
  const float m = ammo.mass;
  const float d = ammo.drag;
  const float l = ammo.lift;

  // --- ЕТАП 1: Розрахунок часу польоту (метод Кардано) ---
  const float a = d * g_gravity * m - 2.0f * d * d * l * speed;
  const float b = -3.0f * g_gravity * m * m + 3.0f * d * l * m * speed;
  const float c = 6.0f * m * m * altitude;

  float t = 0.0f;

  if (std::fabs(a) < std::numeric_limits<float>::epsilon()) {
    t = std::sqrt(2.0f * altitude / g_gravity);
  }
  else {
    const float p = -b * b / (3.0f * a * a);
    const float q = (2.0f * b * b * b) / (27.0f * a * a * a) + c / a;

    if (p >= 0.0f) {
      t = std::sqrt(2.0f * altitude / g_gravity);
    }
    else {
      const float arg = 3.0f * q / (2.0f * p) * std::sqrt(-3.0f / p);
      if (std::fabs(arg) > 1.0f) {
        t = std::sqrt(2.0f * altitude / g_gravity);
      }
      else {
        const float phi = std::acos(arg);
        const float calculatedT = 2.0f * std::sqrt(-p / 3.0f) * std::cos((phi + 4.0f * std::numbers::pi_v<float>) / 3.0f) - b / (3.0f * a);
        t = calculatedT > 0.0f ? calculatedT : std::sqrt(2.0f * altitude / g_gravity);
      }
    }
  }

  // --- ЕТАП 2: Розрахунок горизонтальної дистанції (ряд Тейлора) ---
  const float l2 = l * l;
  const float l4 = l2 * l2;

  const float t2 = t * t;
  const float t3 = t2 * t;
  const float t4 = t3 * t;
  const float t5 = t4 * t;

  const float d2 = d * d;
  const float d3 = d2 * d;
  const float d4 = d3 * d;

  const float m2 = m * m;
  const float m3 = m2 * m;
  const float m4 = m3 * m;

  const float denomFactor = (1.0f + l2) * (1.0f + l2);

  const float hDist =
    speed * t - (d * t2 * speed) / (2.0f * m) + (t3 * (6.0f * d * g_gravity * l * m - 6.0f * d2 * (-1.0f + l2) * speed)) / (36.0f * m2) +
    (t4 *
     (-6.0f * d2 * g_gravity * l * (1.0f + l2 + l4) * m + 3.0f * d3 * l2 * (1.0f + l2) * speed + 6.0f * d3 * l4 * (1.0f + l2) * speed)) /
      (36.0f * denomFactor * m3) +
    (t5 * (3.0f * d3 * g_gravity * l2 * l * m - 3.0f * d4 * l2 * (1.0f + l2) * speed)) / (36.0f * (1.0f + l2) * m4);

  return {.flightTime = t, .hDistance = hDist};
}

}  // namespace BallisticApp::solvers

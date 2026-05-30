#include "BallisticApp/utils/MathUtils.h"
#include <cmath>
#include <numbers>

namespace BallisticApp::Math {

// довжина вектора (hypot)
float length(Coord c)
{
  return std::hypot(c.x, c.y);
}

// одиничний вектор
Coord normalize(Coord c)
{
  const float len = BallisticApp::Math::length(c);

  if (len < 1e-6f) {
    return Coord{0.0f, 0.0f};
  }
  return c / len;
}

// ------------------------------------------------------------
// Нормалізація кута до [-PI, PI]
// ------------------------------------------------------------
float normalizeAngle(float a)
{
  constexpr float pi = std::numbers::pi_v<float>;
  constexpr float two_pi = 2.0f * pi;

  a = std::fmod(a, two_pi);

  if (a > pi) {
    a -= two_pi;
  }
  else if (a < -pi) {
    a += two_pi;
  }

  return a;
}

}  // namespace BallisticApp::Math

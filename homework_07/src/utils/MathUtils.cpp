#include "ballistic_app/utils/MathUtils.h"
#include <cmath>

namespace BallisticApp::Math {

// довжина вектора (hypot)
float length(Coord c)
{
  return std::hypot(c.x, c.y);
}

// одиничний вектор
Coord normalize(Coord c)
{
  float len = BallisticApp::Math::length(c);
  if (len < 1e-12f)
    return Coord{0.0f, 0.0f};
  return c / len;
}

// ------------------------------------------------------------
// Нормалізація кута до [-PI, PI]
// ------------------------------------------------------------
float normalizeAngle(float a)
{
  while (a > M_PI)
    a -= 2.0f * (float)M_PI;
  while (a < -M_PI)
    a += 2.0f * (float)M_PI;
  return a;
}

}  // namespace BallisticApp::Math

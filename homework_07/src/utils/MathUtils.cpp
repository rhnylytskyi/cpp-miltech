#include "ballistic_app/utils/MathUtils.hpp"
#include <cmath>

namespace BallisticApp::Math {

float length(Coord c)
{
  return std::hypot(c.x, c.y);
}

Coord normalize(Coord c)
{
  float len = BallisticApp::Math::length(c);
  if (len < 1e-12f)
    return Coord{0.0f, 0.0f};
  return c / len;
}

float normalizeAngle(float a)
{
  while (a > M_PI)
    a -= 2.0f * (float)M_PI;
  while (a < -M_PI)
    a += 2.0f * (float)M_PI;
  return a;
}

}  // namespace BallisticApp::Math

#pragma once

#include <cmath>
#include <numbers>

namespace BallisticApp::Math {

inline float normalizeAngle(float angle) noexcept
{
  constexpr float PI = std::numbers::pi_v<float>;
  constexpr float TWO_PI = 2.0f * PI;

  angle = std::fmod(angle, TWO_PI);

  if (angle > PI)
    angle -= TWO_PI;
  if (angle <= -PI)
    angle += TWO_PI;

  return angle;
}

}  // namespace BallisticApp::Math

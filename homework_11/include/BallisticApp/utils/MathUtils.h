#pragma once

#include <cmath>
#include <numbers>

namespace BallisticApp {

/**
 * @brief Structure representing 2D Cartesian coordinates and vector operations.
 */
struct Coord {
  float x{0.0f};
  float y{0.0f};

  [[nodiscard]] float length() const noexcept { return std::sqrt(x * x + y * y); }

  [[nodiscard]] float lengthSquared() const noexcept { return x * x + y * y; }

  [[nodiscard]] float distanceTo(const Coord& other) const noexcept
  {
    const float dx = other.x - x;
    const float dy = other.y - y;
    return std::sqrt(dx * dx + dy * dy);
  }

  [[nodiscard]] Coord normalize() const noexcept
  {
    const float len = length();
    if (len < 1e-6f) {
      return {0.0f, 0.0f};
    }
    return {x / len, y / len};
  }

  [[nodiscard]] Coord operator+(const Coord& other) const noexcept { return {x + other.x, y + other.y}; }

  [[nodiscard]] Coord operator-(const Coord& other) const noexcept { return {x - other.x, y - other.y}; }

  [[nodiscard]] Coord operator*(float scalar) const noexcept { return {x * scalar, y * scalar}; }

  [[nodiscard]] Coord operator/(float scalar) const noexcept
  {
    if (std::fabs(scalar) < 1e-6f) {
      return {0.0f, 0.0f};  // Fail-safe behavior for real-time control safety
    }
    return {x / scalar, y / scalar};
  }

  [[nodiscard]] bool operator==(const Coord& other) const noexcept
  {
    return std::fabs(x - other.x) < 1e-5f && std::fabs(y - other.y) < 1e-5f;
  }

  [[nodiscard]] bool operator!=(const Coord& other) const noexcept { return !(*this == other); }
};

}  // namespace BallisticApp

namespace BallisticApp::Math {

/**
 * @brief Normalizes any heading angle to the range (-PI, PI] radians.
 */
[[nodiscard]] inline float normalizeAngle(float angle) noexcept
{
  constexpr float kPi = std::numbers::pi_v<float>;
  constexpr float kTwoPi = 2.0f * kPi;

  angle = std::fmod(angle, kTwoPi);

  if (angle > kPi) {
    angle -= kTwoPi;
  }
  if (angle <= -kPi) {
    angle += kTwoPi;
  }

  return angle;
}

}  // namespace BallisticApp::Math

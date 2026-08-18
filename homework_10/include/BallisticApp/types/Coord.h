#pragma once

#include <cmath>
#include <format>

namespace BallisticApp {

/**
 * @brief 2D Vector coordinate structure for ballistic and physical simulation movement calculations.
 */
struct Coord {
  float x{0.0f};
  float y{0.0f};

  /** @brief Vector addition. */
  constexpr Coord operator+(const Coord& other) const noexcept { return Coord{x + other.x, y + other.y}; }

  /** @brief Vector subtraction. */
  constexpr Coord operator-(const Coord& other) const noexcept { return Coord{x - other.x, y - other.y}; }

  /** @brief Vector multiplication by scalar. */
  constexpr Coord operator*(float s) const noexcept { return Coord{x * s, y * s}; }

  /**
   * @brief Vector division by scalar with safety threshold check.
   * Fully inline-optimized to support constexpr compilation.
   */
  constexpr Coord operator/(float s) const noexcept
  {
    const float absS = s < 0.0f ? -s : s;
    if (absS < 1e-12f)
      return Coord{0.0f, 0.0f};
    return Coord{x / s, y / s};
  }

  /** @brief Vector equality comparison with floating-point epsilon precision. */
  bool operator==(const Coord& other) const noexcept { return (std::abs(x - other.x) < 1e-5f) && (std::abs(y - other.y) < 1e-5f); }

  /**
   * @brief Calculates the actual length (magnitude) of the vector.
   * @note Uses std::sqrt, which is a slow CPU operation.
   */
  float length() const noexcept { return std::sqrt(x * x + y * y); }

  /**
   * @brief Calculates the actual distance to another coordinate point.
   * Leverages internal vector subtraction and magnitude calculation.
   */
  float distanceTo(const Coord& other) const noexcept { return (*this - other).length(); }

  /**
   * @brief Calculates the squared length of the vector.
   * @note Fast alternative to length(), skips std::sqrt. Ideal for proximity checks.
   */
  constexpr float lengthSquared() const noexcept { return x * x + y * y; }

  /**
   * @brief Returns a normalized unit vector (length equals 1.0).
   * Uses fast inverse multiplication optimization.
   */
  Coord normalize() const noexcept
  {
    float len = length();
    if (len < 1e-12f)
      return Coord{0.0f, 0.0f};

    float invLen = 1.0f / len;
    return Coord{x * invLen, y * invLen};
  }

  /**
   * @brief Calculates the scalar dot product between two vectors.
   * Useful for:
   * - Direction validation (positive = ahead, negative = behind)
   * - Angle calculations (dot product of normalized vectors equals cos(angle))
   * - Linear projection tracking
   */
  constexpr float dot(const Coord& other) const noexcept { return x * other.x + y * other.y; }
};

/** @brief Free operator to support global syntax: scalar * Coord. */
constexpr Coord operator*(float s, const Coord& c) noexcept
{
  return c * s;
}

}  // namespace BallisticApp

namespace std {

/**
 * @brief Custom std::formatter specialization for formatted printing of BallisticApp::Coord.
 */
template <>
struct formatter<BallisticApp::Coord> : formatter<std::string_view> {
  auto format(const BallisticApp::Coord& c, std::format_context& ctx) const
  {
    return formatter<std::string_view>::format(std::format("[{:.2f}; {:.2f}]", c.x, c.y), ctx);
  }
};

}  // namespace std

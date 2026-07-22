#pragma once
#include <cmath>
#include <stdexcept>

namespace BallisticApp {

struct Coord {
  float x{0.0f};
  float y{0.0f};

  inline float length() const { return std::sqrt(x * x + y * y); }

  inline float lengthSquared() const { return x * x + y * y; }

  inline float distanceTo(const Coord& other) const
  {
    float dx = other.x - x;
    float dy = other.y - y;
    return std::sqrt(dx * dx + dy * dy);
  }

  inline Coord normalize() const
  {
    float len = length();
    if (len < 1e-6f) {
      return {0.0f, 0.0f};
    }
    return {x / len, y / len};
  }

  inline Coord operator+(const Coord& other) const { return {x + other.x, y + other.y}; }

  inline Coord operator-(const Coord& other) const { return {x - other.x, y - other.y}; }

  inline Coord operator*(float scalar) const { return {x * scalar, y * scalar}; }

  inline Coord operator/(float scalar) const
  {
    if (std::fabs(scalar) < 1e-6f) {
      throw std::runtime_error("Coord Error: Division by zero scalar!");
    }
    return {x / scalar, y / scalar};
  }

  inline bool operator==(const Coord& other) const { return std::fabs(x - other.x) < 1e-5f && std::fabs(y - other.y) < 1e-5f; }

  inline bool operator!=(const Coord& other) const { return !(*this == other); }
};

}  // namespace BallisticApp

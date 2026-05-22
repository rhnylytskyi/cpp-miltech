#pragma once

#include <cmath>

namespace BallisticApp {
struct Coord {
  float x;
  float y;

  Coord operator+(const Coord& other) const { return Coord{x + other.x, y + other.y}; }
  Coord operator-(const Coord& other) const { return Coord{x - other.x, y - other.y}; }
  Coord operator*(float s) const { return Coord{x * s, y * s}; }
  Coord operator/(float s) const
  {
    if (std::fabs(s) < 1e-12f)
      return Coord{0.0f, 0.0f};
    return Coord{x / s, y / s};
  }
  bool operator==(const Coord& other) const { return (std::fabs(x - other.x) < 1e-5f) && (std::fabs(y - other.y) < 1e-5f); }
};
}  // namespace BallisticApp
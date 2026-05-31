#pragma once

#include <cmath>
#include <format>

namespace BallisticApp {

struct Coord {
  float x{0.0f};
  float y{0.0f};

  constexpr Coord operator+(const Coord& other) const noexcept { return Coord{x + other.x, y + other.y}; }
  constexpr Coord operator-(const Coord& other) const noexcept { return Coord{x - other.x, y - other.y}; }
  constexpr Coord operator*(float s) const noexcept { return Coord{x * s, y * s}; }
  constexpr Coord operator/(float s) const noexcept
  {
    if (std::abs(s) < 1e-12f)
      return Coord{0.0f, 0.0f};
    return Coord{x / s, y / s};
  }
  bool operator==(const Coord& other) const noexcept { return (std::abs(x - other.x) < 1e-5f) && (std::abs(y - other.y) < 1e-5f); }

  // Довжина вектора
  float length() const noexcept { return std::sqrt(x * x + y * y); }

  // Повертає новий нормалізований вектор (одиничної довжини)
  Coord normalize() const noexcept
  {
    float len = length();
    if (len < 1e-12f)
      return Coord{0.0f, 0.0f};
    return Coord{x / len, y / len};
  }
};

// Вільний оператор для підтримки синтаксису: скаляр * Coord
constexpr Coord operator*(float s, const Coord& c) noexcept
{
  return c * s;
}

}  // namespace BallisticApp

namespace std {

template <>
struct formatter<BallisticApp::Coord> : formatter<std::string_view> {
  auto format(const BallisticApp::Coord& c, std::format_context& ctx) const
  {
    return formatter<std::string_view>::format(std::format("[{:.2f}; {:.2f}]", c.x, c.y), ctx);
  }
};

}  // namespace std
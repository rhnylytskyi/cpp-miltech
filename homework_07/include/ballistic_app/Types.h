#pragma once

#include <cmath>
#include <string>

namespace BallisticApp {

// 1. Координати та математика (базовий тип)
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

// 2. Стан дрону
enum class DroneState {
  STOPPED = 0,       // Дрон стоїть на місці (початковий стан або після гальмування)
  TURNING = 1,       // Дрон розвертається на місці у напрямку точки скидання
  ACCELERATING = 2,  // Дрон розганяється до своєї максимальної швидкості
  MOVING = 3,        // Дрон летить на максимальній робочій швидкості (атака)
  DECELERATING = 4   // Дрон гальмує (якщо потрібно змінити курс або зупинитися)
};

// 3. Параметри боєприпасу
struct AmmoParams {
  std::string name;
  float mass;
  float drag;
  float lift;
};

// 4. Конфігурація дрону
class DroneConfig {
public:
  Coord startPos;
  float altitude;
  float initialDir;
  float attackSpeed;
  float accelPath;
  std::string ammoName;
  float arrayTimeStep;
  float simTimeStep;
  float hitRadius;
  float angularSpeed;
  float turnThreshold;
};

// 5. Крок симуляції
struct SimStep {
  Coord pos;
  float direction;
  DroneState state;  // Якщо хочете сувору типізацію, можна замінити int на DroneState
  int targetIdx;
  Coord dropPoint;
  Coord aimPoint;
  Coord predictedTarget;
};

}  // namespace BallisticApp

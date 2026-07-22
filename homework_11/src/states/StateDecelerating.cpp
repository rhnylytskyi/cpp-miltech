#include "BallisticApp/states/StateDecelerating.h"
#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/utils/MathUtils.h"
#include <cmath>
#include <algorithm>

namespace BallisticApp {

DroneStateType StateDecelerating::execute(float& speed, float& direction, float desiredDir, const DroneConfig& cfg)
{
  const float dt = cfg.timeStep;

  // Розрахунок уповільнення на основі вашої формули з детермінованим коефіцієнтом 3.0f
  float acceleration = 0.0f;
  if (cfg.accelerationPath > 1e-5f) {  // замість accelPath
    acceleration = (cfg.attackSpeed * cfg.attackSpeed) / (3.0f * cfg.accelerationPath);
  }

  const float prevSpeed = speed;
  // Керуємо поточною швидкістю через посилання
  speed = std::clamp(speed - acceleration * dt, 0.0f, prevSpeed);

  // Ваша фірмова зворотна залежність маневреності від швидкості
  float frac = 1.0f - (speed / cfg.attackSpeed);
  float effectiveAngularSpeed = cfg.angularSpeed * frac;

  // Поворот з урахуванням адаптивної кутової швидкості
  const float deltaAngle = Math::normalizeAngle(desiredDir - direction);
  const float maxTurnThisStep = effectiveAngularSpeed * dt;
  const float actualTurn = std::clamp(deltaAngle, -maxTurnThisStep, maxTurnThisStep);

  // Керуємо напрямком через посилання
  direction = Math::normalizeAngle(direction + actualTurn);

  // Логіка повної зупинки
  if (speed <= 0.0f) {
    speed = 0.0f;

    if (std::fabs(Math::normalizeAngle(desiredDir - direction)) > cfg.turnThreshold) {
      return DroneStateType::TURNING;
    }
    return DroneStateType::STOPPED;
  }

  return DroneStateType::DECELERATING;
}

DroneStateType StateDecelerating::getType() const
{
  return DroneStateType::DECELERATING;
}

}  // namespace BallisticApp

#include "BallisticApp/states/StateAccelerating.h"
#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/utils/MathUtils.h"
#include <cmath>
#include <algorithm>

namespace BallisticApp {

DroneStateType StateAccelerating::execute(float& speed, float& direction, float desiredDir, const DroneConfig& cfg)
{
  const float dt = cfg.timeStep;
  const float deltaAngle = Math::normalizeAngle(desiredDir - direction);

  // Якщо ціль під час розгону різко змістилася — ініціюємо гальмування для безпечного маневру
  if (std::fabs(deltaAngle) > cfg.turnThreshold && speed > 0.01f) {
    return DroneStateType::DECELERATING;
  }

  // Розрахунок прискорення на основі вашого детермінованого коефіцієнта 3.0f
  float acceleration = 0.0f;
  if (cfg.accelerationPath > 1e-5f) {  // замість accelPath
    acceleration = (cfg.attackSpeed * cfg.attackSpeed) / (3.0f * cfg.accelerationPath);
  }

  // Нарощуємо швидкість через посилання
  speed = std::clamp(speed + acceleration * dt, 0.0f, cfg.attackSpeed);

  // Плавне підвертання на ціль під час розгону
  const float maxTurnThisStep = cfg.angularSpeed * dt;
  const float actualTurn = std::clamp(deltaAngle, -maxTurnThisStep, maxTurnThisStep);
  direction = Math::normalizeAngle(direction + actualTurn);

  // При досягненні максимальної швидкості переходимо в режим маршу
  if (speed >= cfg.attackSpeed) {
    return DroneStateType::MOVING;
  }

  return DroneStateType::ACCELERATING;
}

DroneStateType StateAccelerating::getType() const
{
  return DroneStateType::ACCELERATING;
}

}  // namespace BallisticApp

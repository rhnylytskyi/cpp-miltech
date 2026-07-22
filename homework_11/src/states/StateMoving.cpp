#include "BallisticApp/states/StateMoving.h"
#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/utils/MathUtils.h"
#include <cmath>
#include <algorithm>

namespace BallisticApp {

DroneStateType StateMoving::execute(float& speed, float& direction, float desiredDir, const DroneConfig& cfg)
{
  const float dt = cfg.timeStep;
  const float deltaAngle = Math::normalizeAngle(desiredDir - direction);

  // Якщо відхилення занадто велике — негайно переходимо до гальмування для маневру
  if (std::fabs(deltaAngle) > cfg.turnThreshold) {
    return DroneStateType::DECELERATING;
  }

  // Плавне підвертання на курс під час маршу
  const float maxTurnThisStep = cfg.angularSpeed * dt;
  const float actualTurn = std::clamp(deltaAngle, -maxTurnThisStep, maxTurnThisStep);
  direction = Math::normalizeAngle(direction + actualTurn);

  // Підтримуємо максимальну швидкість атаки
  speed = cfg.attackSpeed;

  return DroneStateType::MOVING;
}

DroneStateType StateMoving::getType() const
{
  return DroneStateType::MOVING;
}

}  // namespace BallisticApp

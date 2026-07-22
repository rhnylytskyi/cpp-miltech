#include "BallisticApp/states/StateTurning.h"
#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/utils/MathUtils.h"
#include <cmath>
#include <algorithm>

namespace BallisticApp {

DroneStateType StateTurning::execute(float& speed, float& direction, float desiredDir, const DroneConfig& cfg)
{
  const float dt = cfg.timeStep;
  speed = 0.0f;  // При розвороті на місці лінійна швидкість відсутня

  // Обчислення похибки курсу
  const float deltaAngle = Math::normalizeAngle(desiredDir - direction);

  // Максимальний кут повороту за поточний крок часу телеметрії
  const float maxTurnThisStep = cfg.angularSpeed * dt;
  const float actualTurn = std::clamp(deltaAngle, -maxTurnThisStep, maxTurnThisStep);

  // Змінюємо напрямок дрона напряму через посилання
  direction = Math::normalizeAngle(direction + actualTurn);

  // Ваша фірмова підвищена точність (половина порогу) для стабілізації курсу
  const float deviation = std::fabs(Math::normalizeAngle(desiredDir - direction));
  if (deviation <= (cfg.turnThreshold * 0.5f)) {
    return DroneStateType::ACCELERATING;
  }

  return DroneStateType::TURNING;
}

DroneStateType StateTurning::getType() const
{
  return DroneStateType::TURNING;
}

}  // namespace BallisticApp

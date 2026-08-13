#include "BallisticApp/states/StateMoving.h"
#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/utils/MathUtils.h"
#include <cmath>
#include <algorithm>

namespace BallisticApp::states {

DroneStateType StateMoving::execute(float& speed, float& direction, float desiredDir, const DroneConfig& cfg)
{
  const float dt = cfg.timeStep;
  const float deltaAngle = Math::normalizeAngle(desiredDir - direction);

  // If alignment deviation exceeds acceptable threshold, drop speed to prioritize vector authority
  if (std::fabs(deltaAngle) > cfg.turnThreshold) {
    return DroneStateType::DECELERATING;
  }

  // Execute subtle continuous flight path adjustments during high-speed cruise
  const float maxTurnThisStep = cfg.angularSpeed * dt;
  const float actualTurn = std::clamp(deltaAngle, -maxTurnThisStep, maxTurnThisStep);
  direction = Math::normalizeAngle(direction + actualTurn);

  // Maintain optimized target profile speed
  speed = cfg.attackSpeed;

  return DroneStateType::MOVING;
}

DroneStateType StateMoving::getType() const
{
  return DroneStateType::MOVING;
}

}  // namespace BallisticApp::states

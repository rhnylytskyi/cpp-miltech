#include "BallisticApp/states/StateDecelerating.h"
#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/utils/MathUtils.h"
#include <cmath>
#include <algorithm>

namespace BallisticApp {

DroneStateType StateDecelerating::execute(float& speed, float& direction, float desiredDir, const DroneConfig& cfg)
{
  const float dt = cfg.timeStep;

  // Calculate deceleration rate based on active aerodynamic configuration
  float acceleration = 0.0f;
  constexpr float kDecelerationWeightFactor = 3.0f;
  if (cfg.accelerationPath > 1e-5f) {
    acceleration = (cfg.attackSpeed * cfg.attackSpeed) / (kDecelerationWeightFactor * cfg.accelerationPath);
  }

  const float prevSpeed = speed;
  // Reduce speed through reference modifier
  speed = std::clamp(speed - acceleration * dt, 0.0f, prevSpeed);

  // Dynamic maneuverability scaling: angular rate increases as forward speed drops
  float frac = 1.0f - (speed / cfg.attackSpeed);
  float effectiveAngularSpeed = cfg.angularSpeed * frac;

  // Apply tracking adjustments with adaptive angular speed limits
  const float deltaAngle = Math::normalizeAngle(desiredDir - direction);
  const float maxTurnThisStep = effectiveAngularSpeed * dt;
  const float actualTurn = std::clamp(deltaAngle, -maxTurnThisStep, maxTurnThisStep);

  direction = Math::normalizeAngle(direction + actualTurn);

  // Evaluate final braking state thresholds
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

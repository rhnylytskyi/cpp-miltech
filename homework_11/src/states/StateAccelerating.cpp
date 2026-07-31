#include "BallisticApp/states/StateAccelerating.h"
#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/utils/MathUtils.h"
#include <cmath>
#include <algorithm>

namespace BallisticApp::states {

DroneStateType StateAccelerating::execute(float& speed, float& direction, float desiredDir, const DroneConfig& cfg)
{
  const float dt = cfg.timeStep;
  const float deltaAngle = Math::normalizeAngle(desiredDir - direction);

  // If target shifts drastically during acceleration, trigger deceleration for safe maneuvering
  if (std::fabs(deltaAngle) > cfg.turnThreshold && speed > 0.01f) {
    return DroneStateType::DECELERATING;
  }

  // Calculate forward acceleration using the ballistic trajectory coefficient
  float acceleration = 0.0f;
  constexpr float kAccelerationWeightFactor = 3.0f;
  if (cfg.accelerationPath > 1e-5f) {
    acceleration = (cfg.attackSpeed * cfg.attackSpeed) / (kAccelerationWeightFactor * cfg.accelerationPath);
  }

  // Smoothly increment drone speed
  speed = std::clamp(speed + acceleration * dt, 0.0f, cfg.attackSpeed);

  // Apply course corrections while accelerating
  const float maxTurnThisStep = cfg.angularSpeed * dt;
  const float actualTurn = std::clamp(deltaAngle, -maxTurnThisStep, maxTurnThisStep);
  direction = Math::normalizeAngle(direction + actualTurn);

  // Transition to cruise profile once target speed is achieved
  if (speed >= cfg.attackSpeed) {
    return DroneStateType::MOVING;
  }

  return DroneStateType::ACCELERATING;
}

DroneStateType StateAccelerating::getType() const
{
  return DroneStateType::ACCELERATING;
}

}  // namespace BallisticApp::states

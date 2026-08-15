#include "BallisticApp/states/StateTurning.h"
#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/utils/MathUtils.h"
#include <cmath>
#include <algorithm>

namespace BallisticApp {

DroneStateType StateTurning::execute(float& speed, float& direction, float desiredDir, const DroneConfig& cfg)
{
  const float dt = cfg.timeStep;
  speed = 0.0f;  // Linear translation is disabled during static axis re-alignment

  const float deltaAngle = Math::normalizeAngle(desiredDir - direction);

  // Restrict tracking response step bounds using telemetry clock synchronization
  const float maxTurnThisStep = cfg.angularSpeed * dt;
  const float actualTurn = std::clamp(deltaAngle, -maxTurnThisStep, maxTurnThisStep);

  direction = Math::normalizeAngle(direction + actualTurn);

  // Enforce a strict hysteresis margin to eliminate target alignment oscillation
  const float deviation = std::fabs(Math::normalizeAngle(desiredDir - direction));
  constexpr float kHysteresisFactor = 0.5f;
  if (deviation <= (cfg.turnThreshold * kHysteresisFactor)) {
    return DroneStateType::ACCELERATING;
  }

  return DroneStateType::TURNING;
}

DroneStateType StateTurning::getType() const
{
  return DroneStateType::TURNING;
}

}  // namespace BallisticApp

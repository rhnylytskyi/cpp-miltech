#include "BallisticApp/mission/FlightController.h"
#include "BallisticApp/utils/MathUtils.h"
#include "BallisticApp/DroneStateType.h"
#include <cmath>
#include <algorithm>

namespace BallisticApp {

dlink::Control FlightController::compute(
  const dlink::Telemetry& tel, float desiredDir, DroneStateType stateType, float turnThreshold, float attackSpeed) const noexcept
{
  // Enforce unified high-performance std::fmod-based angle normalization
  float headingErr = Math::normalizeAngle(desiredDir - tel.dir);
  float turnRate = std::clamp(headingErr / m_turnSaturationRad, -1.0f, 1.0f);

  float targetSpeed = tel.speed;  // Default fallback profile

  if (stateType == DroneStateType::MOVING || stateType == DroneStateType::ACCELERATING) {
    float speedFrac = 1.0f;
    float absErr = std::fabs(headingErr);

    // Smoothly scale down target cruise velocity during aggressive maneuvers
    if (absErr > turnThreshold) {
      float span = std::max(1e-3f, m_sharpTurnRad - turnThreshold);
      float t = std::clamp((absErr - turnThreshold) / span, 0.0f, 1.0f);
      speedFrac = 1.0f - t * (1.0f - m_minSpeedFrac);
    }

    targetSpeed = attackSpeed * speedFrac;
  }
  else if (stateType == DroneStateType::DECELERATING || stateType == DroneStateType::STOPPED) {
    targetSpeed = 0.0f;
  }

  float speedErr = targetSpeed - tel.speed;

  // Under active acceleration with large velocity error, request full forward throttle
  if ((stateType == DroneStateType::ACCELERATING || stateType == DroneStateType::MOVING) && speedErr > 0.05f) {
    return dlink::Control{1.0f, turnRate};
  }

  // Under braking state profiles, request absolute full reverse acceleration
  if (stateType == DroneStateType::DECELERATING) {
    return dlink::Control{-1.0f, turnRate};
  }

  // Linear proportional velocity maintenance window logic
  float band = std::max(0.1f, tel.speed * m_accelBand);
  float accel = std::clamp(speedErr / band, -1.0f, 1.0f);

  return dlink::Control{accel, turnRate};
}

}  // namespace BallisticApp

#include "BallisticApp/control/FlightController.h"
#include "BallisticApp/DroneStateType.h"
#include "BallisticApp/utils/MathUtils.h"
#include <algorithm>
#include <cmath>

namespace BallisticApp {

dlink::Control FlightController::compute(
  const dlink::Telemetry& tel, float desiredDir, DroneStateType stateType, float turnThreshold, float attackSpeed) const noexcept
{
  float headingErr = Math::normalizeAngle(desiredDir - tel.dir);
  float turnRate = std::clamp(headingErr / m_turnSaturationRad, -1.0f, 1.0f);

  float targetSpeed = tel.speed;

  if (stateType == DroneStateType::MOVING || stateType == DroneStateType::ACCELERATING) {
    float speedFrac = 1.0f;
    float absErr = std::fabs(headingErr);

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

  if ((stateType == DroneStateType::ACCELERATING || stateType == DroneStateType::MOVING) && speedErr > 0.05f) {
    return dlink::Control{1.0f, turnRate};
  }

  if (stateType == DroneStateType::DECELERATING) {
    return dlink::Control{-1.0f, turnRate};
  }

  float band = std::max(0.1f, tel.speed * m_accelBand);
  float accel = std::clamp(speedErr / band, -1.0f, 1.0f);

  return dlink::Control{accel, turnRate};
}

}  // namespace BallisticApp

#include "BallisticApp/states/StateStopped.h"
#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/utils/MathUtils.h"
#include <cmath>

namespace BallisticApp {

DroneStateType StateStopped::execute(float& speed, float& direction, float desiredDir, const DroneConfig& cfg)
{
  // Enforce zero linear velocity in a static position profile
  speed = 0.0f;

  // Evaluate standard intercept heading error alignment
  const float deltaAngle = Math::normalizeAngle(desiredDir - direction);

  // Pivot on the spot if misalignment exceeds active threshold limits
  if (std::fabs(deltaAngle) > cfg.turnThreshold) {
    return DroneStateType::TURNING;
  }

  // Trigger aggressive acceleration profile if flight path is aligned
  return DroneStateType::ACCELERATING;
}

DroneStateType StateStopped::getType() const
{
  return DroneStateType::STOPPED;
}

}  // namespace BallisticApp

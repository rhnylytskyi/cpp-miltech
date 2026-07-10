#include "BallisticApp/states/StateStopped.h"
#include "BallisticApp/utils/MathUtils.h"
#include <cmath>

namespace BallisticApp {

DroneStateType StateStopped::execute(MissionContext& ctx)
{
  ctx.lastDeltaPath = 0.0f;
  ctx.speed = 0.0f;

  const float deltaAngle = Math::normalizeAngle(ctx.desiredDir - ctx.direction);

  if (std::fabs(deltaAngle) > ctx.cfg.turnThreshold) {
    return DroneStateType::TURNING;
  }

  return DroneStateType::ACCELERATING;
}

DroneStateType StateStopped::getType() const
{
  return DroneStateType::STOPPED;
}

}  // namespace BallisticApp

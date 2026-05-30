#include "BallisticApp/states/StateStopped.h"
#include "BallisticApp/MissionContext.h"
#include "BallisticApp/utils/MathUtils.h"
#include <cmath>

namespace BallisticApp {

DroneState StateStopped::execute(MissionContext& ctx)
{
  ctx.lastDeltaPath = 0.0f;
  float deltaAngle = Math::normalizeAngle(ctx.desiredDir - ctx.direction);

  if (std::fabs(deltaAngle) > ctx.cfg.turnThreshold) {
    return DroneState::TURNING;
  }
  ctx.direction = ctx.desiredDir;
  return DroneState::ACCELERATING;
}

DroneState StateStopped::getType() const
{
  return DroneState::STOPPED;
}

}  // namespace BallisticApp

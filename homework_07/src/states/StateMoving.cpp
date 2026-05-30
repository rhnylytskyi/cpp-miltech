#include "BallisticApp/states/StateMoving.h"
#include "BallisticApp/MissionContext.h"
#include "BallisticApp/utils/MathUtils.h"
#include <cmath>

namespace BallisticApp {

DroneState StateMoving::execute(MissionContext& ctx)
{
  float dt = ctx.cfg.simTimeStep;
  float deltaAngle = Math::normalizeAngle(ctx.desiredDir - ctx.direction);

  if (std::fabs(deltaAngle) > ctx.cfg.turnThreshold) {
    return DroneState::DECELERATING;
  }

  ctx.direction = ctx.desiredDir;
  ctx.lastDeltaPath = ctx.speed * dt;
  return DroneState::MOVING;
}

DroneState StateMoving::getType() const
{
  return DroneState::MOVING;
}

}  // namespace BallisticApp

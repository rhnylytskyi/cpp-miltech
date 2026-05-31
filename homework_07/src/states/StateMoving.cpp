#include "BallisticApp/states/StateMoving.h"
#include "BallisticApp/MissionContext.h"
#include "BallisticApp/utils/MathUtils.h"
#include <cmath>

namespace BallisticApp {

DroneStateType StateMoving::execute(MissionContext& ctx)
{
  float dt = ctx.cfg.simTimeStep;
  float deltaAngle = Math::normalizeAngle(ctx.desiredDir - ctx.direction);

  if (std::fabs(deltaAngle) > ctx.cfg.turnThreshold) {
    return DroneStateType::DECELERATING;
  }

  ctx.direction = ctx.desiredDir;
  ctx.lastDeltaPath = ctx.speed * dt;
  return DroneStateType::MOVING;
}

}  // namespace BallisticApp

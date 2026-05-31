#include "BallisticApp/states/StateStopped.h"
#include "BallisticApp/types/DroneStateType.h"
#include "BallisticApp/MissionContext.h"
#include "BallisticApp/utils/MathUtils.h"
#include <cmath>

namespace BallisticApp {

DroneStateType StateStopped::execute(MissionContext& ctx)
{
  ctx.lastDeltaPath = 0.0f;
  float deltaAngle = Math::normalizeAngle(ctx.desiredDir - ctx.direction);

  if (std::fabs(deltaAngle) > ctx.cfg.turnThreshold) {
    return DroneStateType::TURNING;
  }
  ctx.direction = ctx.desiredDir;
  return DroneStateType::ACCELERATING;
}

}  // namespace BallisticApp

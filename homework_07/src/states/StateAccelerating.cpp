#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/states/StateAccelerating.h"
#include "BallisticApp/MissionContext.h"
#include "BallisticApp/types/DroneStateType.h"
#include "BallisticApp/utils/MathUtils.h"
#include <cmath>
#include <algorithm>

namespace BallisticApp {

DroneStateType StateAccelerating::execute(MissionContext& ctx)
{
  float dt = ctx.cfg.simTimeStep;
  float deltaAngle = Math::normalizeAngle(ctx.desiredDir - ctx.direction);
  bool isTurningRequired = (std::fabs(deltaAngle) > ctx.cfg.turnThreshold);

  if (isTurningRequired && ctx.speed > 0.01f) {
    return DroneStateType::DECELERATING;
  }

  if (!isTurningRequired) {
    ctx.direction = ctx.desiredDir;
  }

  const float acceleration = (ctx.cfg.attackSpeed * ctx.cfg.attackSpeed) / (2.0f * ctx.cfg.accelPath);
  const float prevSpeed = ctx.speed;

  ctx.speed = std::clamp(ctx.speed + acceleration * dt, 0.0f, ctx.cfg.attackSpeed);
  ctx.lastDeltaPath = (prevSpeed + ctx.speed) / 2.0f * dt;

  if (ctx.speed >= ctx.cfg.attackSpeed) {
    return DroneStateType::MOVING;
  }
  return DroneStateType::ACCELERATING;
}

}  // namespace BallisticApp

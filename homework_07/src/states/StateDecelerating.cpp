#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/states/StateDecelerating.h"
#include "BallisticApp/MissionContext.h"
#include "BallisticApp/types/DroneStateType.h"
#include <cmath>
#include <algorithm>

namespace BallisticApp {

DroneStateType StateDecelerating::execute(MissionContext& ctx)
{
  const float dt = ctx.cfg.simTimeStep;

  float acceleration = (ctx.cfg.attackSpeed * ctx.cfg.attackSpeed) / (2.0f * ctx.cfg.accelPath);
  if (ctx.speed < acceleration * dt) {
    acceleration = ctx.speed / dt;
  }

  const float prevSpeed = ctx.speed;
  ctx.speed = std::clamp(ctx.speed - acceleration * dt, 0.0f, ctx.speed);
  ctx.lastDeltaPath = ((prevSpeed + ctx.speed) / 2.0f) * dt;

  if (ctx.speed <= 0.0f) {
    return DroneStateType::STOPPED;
  }

  return DroneStateType::DECELERATING;
}

DroneStateType StateDecelerating::getType() const
{
  return DroneStateType::DECELERATING;
}

}  // namespace BallisticApp

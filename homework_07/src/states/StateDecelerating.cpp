#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/states/StateDecelerating.h"
#include "BallisticApp/MissionContext.h"
#include <cmath>
#include <algorithm>

namespace BallisticApp {

DroneState StateDecelerating::execute(MissionContext& ctx)
{
  float dt = ctx.cfg.simTimeStep;
  const float acceleration = (ctx.cfg.attackSpeed * ctx.cfg.attackSpeed) / (2.0f * ctx.cfg.accelPath);
  const float prevSpeed = ctx.speed;

  ctx.speed = std::clamp(ctx.speed - acceleration * dt, 0.0f, ctx.speed);
  ctx.lastDeltaPath = (prevSpeed + ctx.speed) / 2.0f * dt;

  if (ctx.speed <= 0.0f) {
    return DroneState::STOPPED;
  }
  return DroneState::DECELERATING;
}

DroneState StateDecelerating::getType() const
{
  return DroneState::DECELERATING;
}

}  // namespace BallisticApp

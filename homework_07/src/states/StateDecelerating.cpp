#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/states/StateDecelerating.h"
#include "BallisticApp/states/StateStopped.h"
#include "BallisticApp/DroneContext.h"
#include <cmath>
#include <algorithm>

namespace BallisticApp {

std::unique_ptr<IDroneState> StateDecelerating::execute(DroneContext& ctx, float dt)
{
  const float acceleration = (ctx.cfg.attackSpeed * ctx.cfg.attackSpeed) / (2.0f * ctx.cfg.accelPath);
  const float prevSpeed = ctx.speed;

  ctx.speed = std::clamp(ctx.speed - acceleration * dt, 0.0f, ctx.speed);
  ctx.lastDeltaPath = (prevSpeed + ctx.speed) / 2.0f * dt;

  if (ctx.speed <= 0.0f) {
    return std::make_unique<StateStopped>();
  }
  return nullptr;
}

DroneState StateDecelerating::getType() const
{
  return DroneState::DECELERATING;
}

}  // namespace BallisticApp

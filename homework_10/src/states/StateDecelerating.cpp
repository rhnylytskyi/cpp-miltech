#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/utils/MathUtils.h"
#include "BallisticApp/states/StateDecelerating.h"
#include <cmath>
#include <algorithm>

namespace BallisticApp {

DroneStateType StateDecelerating::execute(MissionContext& ctx)
{
  const float dt = ctx.deltaTime;
  const float acceleration = (ctx.cfg.attackSpeed * ctx.cfg.attackSpeed) / (3.0f * ctx.cfg.accelPath);

  const float prevSpeed = ctx.speed;
  ctx.speed = std::clamp(ctx.speed - acceleration * dt, 0.0f, prevSpeed);
  ctx.lastDeltaPath = ((prevSpeed + ctx.speed) / 2.0f) * dt;

  // The slower the speed, the higher the maneuverability of the drone
  float frac = 1.0f - (ctx.speed / ctx.cfg.attackSpeed);
  float effectiveAngularSpeed = ctx.cfg.angularSpeed * frac;

  const float deltaAngle = Math::normalizeAngle(ctx.desiredDir - ctx.direction);
  const float maxTurnThisStep = effectiveAngularSpeed * dt;
  const float actualTurn = std::clamp(deltaAngle, -maxTurnThisStep, maxTurnThisStep);
  ctx.direction = Math::normalizeAngle(ctx.direction + actualTurn);

  if (ctx.speed <= 0.0f) {
    ctx.speed = 0.0f;
    ctx.lastDeltaPath = 0.0f;

    if (std::fabs(Math::normalizeAngle(ctx.desiredDir - ctx.direction)) > ctx.cfg.turnThreshold) {
      return DroneStateType::TURNING;
    }
    return DroneStateType::STOPPED;
  }
  return DroneStateType::DECELERATING;
}

DroneStateType StateDecelerating::getType() const
{
  return DroneStateType::DECELERATING;
}

}  // namespace BallisticApp

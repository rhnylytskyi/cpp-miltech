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
  const float dt = ctx.cfg.simTimeStep;
  const float deltaAngle = Math::normalizeAngle(ctx.desiredDir - ctx.direction);
  const bool isTurningRequired = (std::fabs(deltaAngle) > ctx.cfg.turnThreshold);

  if (isTurningRequired && ctx.speed > 0.01f) {
    ctx.lastDeltaPath = ctx.speed * dt;  // Continue moving by inertia of acceleration
    return DroneStateType::DECELERATING;
  }

  const float maxTurnThisStep = ctx.cfg.angularSpeed * dt;
  const float actualTurn = std::clamp(deltaAngle, -maxTurnThisStep, maxTurnThisStep);
  ctx.direction = Math::normalizeAngle(ctx.direction + actualTurn);

  float acceleration = (ctx.cfg.attackSpeed * ctx.cfg.attackSpeed) / (2.0f * ctx.cfg.accelPath);
  const float speedRemaining = ctx.cfg.attackSpeed - ctx.speed;
  if (speedRemaining < acceleration * dt) {
    acceleration = speedRemaining / dt;
  }

  const float prevSpeed = ctx.speed;
  ctx.speed = std::clamp(ctx.speed + acceleration * dt, 0.0f, ctx.cfg.attackSpeed);
  ctx.lastDeltaPath = ((prevSpeed + ctx.speed) / 2.0f) * dt;

  if (ctx.speed >= ctx.cfg.attackSpeed) {
    return DroneStateType::MOVING;
  }

  return DroneStateType::ACCELERATING;
}

DroneStateType StateAccelerating::getType() const
{
  return DroneStateType::ACCELERATING;
}

}  // namespace BallisticApp

#include "BallisticApp/states/StateMoving.h"
#include "BallisticApp/mission/MissionContext.h"
#include "BallisticApp/utils/MathUtils.h"
#include "BallisticApp/types/DroneStateType.h"
#include <cmath>
#include <algorithm>

namespace BallisticApp {

DroneStateType StateMoving::execute(MissionContext& ctx)
{
  const float dt = ctx.cfg.simTimeStep;
  const float deltaAngle = Math::normalizeAngle(ctx.desiredDir - ctx.direction);

  if (std::fabs(deltaAngle) > ctx.cfg.turnThreshold) {
    ctx.lastDeltaPath = ctx.speed * dt;  // Flying current step by inertia
    return DroneStateType::DECELERATING;
  }

  const float maxTurnThisStep = ctx.cfg.angularSpeed * dt;
  const float actualTurn = std::clamp(deltaAngle, -maxTurnThisStep, maxTurnThisStep);
  ctx.direction = Math::normalizeAngle(ctx.direction + actualTurn);

  ctx.speed = ctx.cfg.attackSpeed;
  ctx.lastDeltaPath = ctx.speed * dt;

  return DroneStateType::MOVING;
}

DroneStateType StateMoving::getType() const
{
  return DroneStateType::MOVING;
}

}  // namespace BallisticApp

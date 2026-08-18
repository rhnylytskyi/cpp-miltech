#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/utils/MathUtils.h"
#include "BallisticApp/states/StateMoving.h"
#include <cmath>
#include <algorithm>

namespace BallisticApp {

DroneStateType StateMoving::execute(MissionContext& ctx)
{
  const float dt = ctx.deltaTime;
  const float deltaAngle = Math::normalizeAngle(ctx.desiredDir - ctx.direction);

  // If the deviation angle is too large — immediately reset speed for maneuvering
  if (std::fabs(deltaAngle) > ctx.cfg.turnThreshold) {
    ctx.lastDeltaPath = ctx.speed * dt;
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

#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/utils/MathUtils.h"
#include "BallisticApp/states/StateTurning.h"
#include <cmath>
#include <algorithm>

namespace BallisticApp {

DroneStateType StateTurning::execute(MissionContext& ctx)
{
  const float dt = ctx.deltaTime;
  ctx.speed = 0.0f;
  ctx.lastDeltaPath = 0.0f;

  const float deltaAngle = Math::normalizeAngle(ctx.desiredDir - ctx.direction);
  const float maxTurnThisStep = ctx.cfg.angularSpeed * dt;
  const float actualTurn = std::clamp(deltaAngle, -maxTurnThisStep, maxTurnThisStep);
  ctx.direction = Math::normalizeAngle(ctx.direction + actualTurn);

  // Використовуємо затиснутий поріг точності (половина turnThreshold), щоб стабілізувати курс
  const float deviation = std::fabs(Math::normalizeAngle(ctx.desiredDir - ctx.direction));
  if (deviation <= (ctx.cfg.turnThreshold * 0.5f)) {
    return DroneStateType::ACCELERATING;
  }
  return DroneStateType::TURNING;
}

DroneStateType StateTurning::getType() const
{
  return DroneStateType::TURNING;
}

}  // namespace BallisticApp

#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/states/StateAccelerating.h"
#include "BallisticApp/utils/MathUtils.h"
#include <cmath>
#include <algorithm>

namespace BallisticApp {

DroneStateType StateAccelerating::execute(MissionContext& ctx)
{
  const float dt = ctx.deltaTime;
  const float deltaAngle = Math::normalizeAngle(ctx.desiredDir - ctx.direction);

  // Якщо під час розгону ціль різко змістилася — ініціюємо гальмування
  if (std::fabs(deltaAngle) > ctx.cfg.turnThreshold && ctx.speed > 0.01f) {
    return DroneStateType::DECELERATING;
  }

  // Розрахунок прискорення з урахуванням детермінованого коефіцієнта 3.0f
  const float acceleration = (ctx.cfg.attackSpeed * ctx.cfg.attackSpeed) / (3.0f * ctx.cfg.accelPath);

  const float prevSpeed = ctx.speed;
  ctx.speed = std::clamp(ctx.speed + acceleration * dt, 0.0f, ctx.cfg.attackSpeed);
  ctx.lastDeltaPath = ((prevSpeed + ctx.speed) / 2.0f) * dt;

  // Плавний поворот в процесі розгону
  const float maxTurnThisStep = ctx.cfg.angularSpeed * dt;
  const float actualTurn = std::clamp(deltaAngle, -maxTurnThisStep, maxTurnThisStep);
  ctx.direction = Math::normalizeAngle(ctx.direction + actualTurn);

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

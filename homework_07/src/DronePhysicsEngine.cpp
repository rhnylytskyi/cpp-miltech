#include "BallisticApp/DronePhysicsEngine.h"
#include "BallisticApp/MissionContext.h"
#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/states/DroneStateRegistry.h"
#include <cmath>

namespace BallisticApp {

void DronePhysicsEngine::update(MissionContext& ctx) const
{
  if (!ctx.currentState)
    return;

  // Розрахунок бажаного напрямку
  ctx.desiredDir = std::atan2(ctx.firePoint.y - ctx.pos.y, ctx.firePoint.x - ctx.pos.x);

  // Виконуємо логіку стану і отримуємо енум наступного стану
  DroneStateType nextStateType = ctx.currentState->execute(ctx);

  // Якщо стан змінився, оновлюємо вказівник та тип у контексті
  if (nextStateType != ctx.currentState->getType()) {
    ctx.currentState = DroneStateRegistry::getState(nextStateType);
  }

  // Оновлюємо позицію
  ctx.pos.x += std::cos(ctx.direction) * ctx.lastDeltaPath;
  ctx.pos.y += std::sin(ctx.direction) * ctx.lastDeltaPath;
}

}  // namespace BallisticApp

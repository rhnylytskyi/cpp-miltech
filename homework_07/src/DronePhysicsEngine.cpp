#include "BallisticApp/DronePhysicsEngine.h"
#include <cmath>

namespace BallisticApp {

DronePhysicsEngine::DronePhysicsEngine(const DroneConfig& config)
  : m_config(config)
{
}

void DronePhysicsEngine::update(MissionContext& ctx, std::unique_ptr<IDroneState>& currentState, const Coord& firePoint) const
{
  // Обчислюємо бажаний напрямок руху до цілі
  ctx.desiredDir = std::atan2(firePoint.y - ctx.pos.y, firePoint.x - ctx.pos.x);

  // Делегуємо логіку поточному стану
  auto nextState = currentState->execute(ctx);
  if (nextState) {
    currentState = std::move(nextState);
  }

  // Змінюємо позицію на основі розрахованого зміщення всередині стану
  ctx.pos.x += std::cos(ctx.direction) * ctx.lastDeltaPath;
  ctx.pos.y += std::sin(ctx.direction) * ctx.lastDeltaPath;
}

}  // namespace BallisticApp

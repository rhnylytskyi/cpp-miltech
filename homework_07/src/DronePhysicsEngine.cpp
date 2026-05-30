#include "BallisticApp/DronePhysicsEngine.h"

namespace BallisticApp {

DronePhysicsEngine::DronePhysicsEngine(const DroneConfig& config)
  : m_config(config)
{
}

void DronePhysicsEngine::update(DroneContext& ctx, std::unique_ptr<IDroneState>& currentState, const Coord& firePoint, float dt) const
{
  // Обчислюємо бажаний напрямок руху до цілі
  ctx.desiredDir = std::atan2(firePoint.y - ctx.pos.y, firePoint.x - ctx.pos.x);

  // Делегуємо логіку поточному стану
  auto nextState = currentState->execute(ctx, dt);
  if (nextState) {
    currentState = std::move(nextState);
  }

  // Змінюємо позицію на основі розрахованого зміщення всередині стану
  ctx.pos.x += std::cos(ctx.direction) * ctx.lastDeltaPath;
  ctx.pos.y += std::sin(ctx.direction) * ctx.lastDeltaPath;
}

}  // namespace BallisticApp

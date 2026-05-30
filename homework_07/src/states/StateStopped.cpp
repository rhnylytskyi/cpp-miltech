#include "BallisticApp/states/StateStopped.h"
#include "BallisticApp/states/StateTurning.h"  // Потрібен, бо ми створюємо його всередині через make_unique
#include "BallisticApp/states/StateAccelerating.h"  // Потрібен аналогічно
#include "BallisticApp/DroneContext.h"  // Тепер повністю підключаємо контекст, бо тут потрібні його поля
#include "BallisticApp/utils/MathUtils.h"
#include <cmath>

namespace BallisticApp {

std::unique_ptr<IDroneState> StateStopped::execute(DroneContext& ctx, [[maybe_unused]] float dt)
{
  ctx.lastDeltaPath = 0.0f;
  float deltaAngle = Math::normalizeAngle(ctx.desiredDir - ctx.direction);

  if (std::fabs(deltaAngle) > ctx.cfg.turnThreshold) {
    return std::make_unique<StateTurning>();
  }
  ctx.direction = ctx.desiredDir;
  return std::make_unique<StateAccelerating>();
}

DroneState StateStopped::getType() const
{
  return DroneState::STOPPED;
}

}  // namespace BallisticApp

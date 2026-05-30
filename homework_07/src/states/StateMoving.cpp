#include "BallisticApp/states/StateMoving.h"
#include "BallisticApp/states/StateDecelerating.h"
#include "BallisticApp/DroneContext.h"
#include "BallisticApp/utils/MathUtils.h"
#include <cmath>

namespace BallisticApp {

std::unique_ptr<IDroneState> StateMoving::execute(DroneContext& ctx, float dt)
{
  float deltaAngle = Math::normalizeAngle(ctx.desiredDir - ctx.direction);

  if (std::fabs(deltaAngle) > ctx.cfg.turnThreshold) {
    return std::make_unique<StateDecelerating>();
  }

  ctx.direction = ctx.desiredDir;
  ctx.lastDeltaPath = ctx.speed * dt;
  return nullptr;
}

DroneState StateMoving::getType() const
{
  return DroneState::MOVING;
}

}  // namespace BallisticApp

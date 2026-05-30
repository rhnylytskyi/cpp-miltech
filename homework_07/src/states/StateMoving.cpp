#include "BallisticApp/states/StateMoving.h"
#include "BallisticApp/states/StateDecelerating.h"
#include "BallisticApp/MissionContext.h"
#include "BallisticApp/utils/MathUtils.h"
#include <cmath>
#include <memory>

namespace BallisticApp {

std::unique_ptr<IDroneState> StateMoving::execute(MissionContext& ctx)
{
  float dt = ctx.cfg.simTimeStep;
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

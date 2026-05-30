#include "BallisticApp/states/StateStopped.h"
#include "BallisticApp/states/StateTurning.h"
#include "BallisticApp/states/StateAccelerating.h"
#include "BallisticApp/MissionContext.h"
#include "BallisticApp/utils/MathUtils.h"
#include <cmath>

namespace BallisticApp {

std::unique_ptr<IDroneState> StateStopped::execute(MissionContext& ctx)
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

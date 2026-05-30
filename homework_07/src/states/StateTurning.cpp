#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/states/StateTurning.h"
#include "BallisticApp/states/StateAccelerating.h"
#include "BallisticApp/MissionContext.h"
#include <cmath>
#include <algorithm>

namespace BallisticApp {

std::unique_ptr<IDroneState> StateTurning::execute(MissionContext& ctx)
{
  float dt = ctx.cfg.simTimeStep;
  ctx.lastDeltaPath = 0.0f;
  float deltaAngle = Math::normalizeAngle(ctx.desiredDir - ctx.direction);
  const float maxTurnThisStep = ctx.cfg.angularSpeed * dt;
  const float actualTurn = std::clamp(deltaAngle, -maxTurnThisStep, maxTurnThisStep);

  ctx.direction = Math::normalizeAngle(ctx.direction + actualTurn);

  if (std::fabs(Math::normalizeAngle(ctx.desiredDir - ctx.direction)) <= ctx.cfg.turnThreshold) {
    ctx.direction = ctx.desiredDir;
    return std::make_unique<StateAccelerating>();
  }
  return nullptr;
}

DroneState StateTurning::getType() const
{
  return DroneState::TURNING;
}

}  // namespace BallisticApp

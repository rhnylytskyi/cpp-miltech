#include "BallisticApp/navigation/DroneAutopilot.h"
#include "BallisticApp/mission/MissionContext.h"
#include "BallisticApp/states/DroneStateRegistry.h"
#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/utils/MathUtils.h"
#include <cmath>

namespace BallisticApp {

void DroneAutopilot::update(MissionContext& ctx) const
{
  if (!ctx.currentState) {
    return;
  }

  // Lead guidance: targeting the predicted intercept point (firePoint)
  // pre-calculated by the fire control computer (FCC).
  const float targetAngle = std::atan2(ctx.firePoint.y - ctx.pos.y, ctx.firePoint.x - ctx.pos.x);

  // FILTERING OF THE COURSE: smoothly compensate for angular deviation
  const float angleDeviation = Math::normalizeAngle(targetAngle - ctx.direction);
  const float smoothingFactor = 0.5f;

  ctx.desiredDir = Math::normalizeAngle(ctx.direction + angleDeviation * smoothingFactor);

  // CONTROL: execute the logic of the current state
  DroneStateType nextStateType = ctx.currentState->execute(ctx);

  if (nextStateType != ctx.currentState->getType()) {
    ctx.currentState = DroneStateRegistry::getState(nextStateType);
  }

  // PHYSICS (EULER-CROMER): integrate coordinates at the end of the time step
  ctx.pos.x += std::cos(ctx.direction) * ctx.lastDeltaPath;
  ctx.pos.y += std::sin(ctx.direction) * ctx.lastDeltaPath;
}

}  // namespace BallisticApp

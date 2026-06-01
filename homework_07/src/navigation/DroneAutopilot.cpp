#include "BallisticApp/navigation/DroneAutopilot.h"
#include "BallisticApp/mission/MissionContext.h"
#include "BallisticApp/states/DroneStateRegistry.h"
#include "BallisticApp/interfaces/IDroneState.h"
#include <cmath>

namespace BallisticApp {

void DroneAutopilot::update(MissionContext& ctx) const
{
  if (!ctx.currentState) {
    return;
  }

  // 1. Navigation: calculation of the target course to the target
  ctx.desiredDir = std::atan2(ctx.firePoint.y - ctx.pos.y, ctx.firePoint.x - ctx.pos.x);

  // 2. Control: state calculates speed and path delta for this tick
  DroneStateType nextStateType = ctx.currentState->execute(ctx);

  // 3. State change (if conditions inside execute() forced the automaton to switch)
  if (nextStateType != ctx.currentState->getType()) {
    ctx.currentState = DroneStateRegistry::getState(nextStateType);
  }

  // 4. Physics (Euler-Cromer method):
  // Integrate coordinates strictly after updating the angle and path delta
  ctx.pos.x += std::cos(ctx.direction) * ctx.lastDeltaPath;
  ctx.pos.y += std::sin(ctx.direction) * ctx.lastDeltaPath;
}

}  // namespace BallisticApp

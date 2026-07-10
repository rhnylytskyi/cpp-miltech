#include "BallisticApp/mission/MissionContext.h"
#include "BallisticApp/interfaces/IDroneState.h"

namespace BallisticApp {

DroneStateType MissionContext::getCurrentStateType() const
{
  return currentState ? currentState->getType() : DroneStateType::STOPPED;
}

}  // namespace BallisticApp

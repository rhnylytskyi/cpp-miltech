#pragma once

#include "BallisticApp/MissionContext.h"
#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/DroneStateType.h"

namespace BallisticApp {

class StateMoving : public IDroneState {
public:
  DroneStateType execute(MissionContext& ctx) override;
};

}  // namespace BallisticApp

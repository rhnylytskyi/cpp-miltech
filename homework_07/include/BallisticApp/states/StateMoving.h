#pragma once

#include "BallisticApp/MissionContext.h"
#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/types/DroneStateType.h"

namespace BallisticApp {

class StateMoving : public IDroneState {
public:
  DroneStateType execute(MissionContext& ctx) override;
  DroneStateType getType() const override; 
};

}  // namespace BallisticApp

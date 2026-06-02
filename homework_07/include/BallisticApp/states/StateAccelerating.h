#pragma once

#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/mission/MissionContext.h"
#include "BallisticApp/types/DroneStateType.h"

namespace BallisticApp {

class StateAccelerating : public IDroneState {
public:
  DroneStateType execute(MissionContext& ctx) override;
  DroneStateType getType() const override; 
};

}  // namespace BallisticApp
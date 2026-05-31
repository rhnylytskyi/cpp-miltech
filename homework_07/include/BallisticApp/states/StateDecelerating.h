#pragma once

#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/types/DroneStateType.h"
#include "BallisticApp/MissionContext.h"

namespace BallisticApp {

class StateDecelerating : public IDroneState {
public:
  DroneStateType execute(MissionContext& ctx) override;
  DroneStateType getType() const override; 
};

}  // namespace BallisticApp
#pragma once

#include "BallisticApp/MissionContext.h"
#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/types/DroneStateType.h"

namespace BallisticApp {

class StateTurning : public IDroneState {
public:
  DroneStateType execute(MissionContext& ctx) override;
};

}  // namespace BallisticApp
#pragma once

#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/MissionContext.h"
#include "BallisticApp/types/DroneStateType.h"

namespace BallisticApp {

class StateStopped : public IDroneState {
public:
  DroneStateType execute(MissionContext& ctx) override;
};

}  // namespace BallisticApp

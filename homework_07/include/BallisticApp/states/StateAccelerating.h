#pragma once

#include "BallisticApp/MissionContext.h"
#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/DroneState.h"

namespace BallisticApp {

class StateAccelerating : public IDroneState {
public:
  DroneState execute(MissionContext& ctx) override;
  DroneState getType() const override;
};

}  // namespace BallisticApp
#pragma once

#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/MissionContext.h"
#include "BallisticApp/DroneState.h"

namespace BallisticApp {

class StateStopped : public IDroneState {
public:
  DroneState execute(MissionContext& ctx) override;
  DroneState getType() const override;
};

}  // namespace BallisticApp

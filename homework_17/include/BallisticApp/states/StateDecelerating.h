#pragma once
#include "BallisticApp/interfaces/IDroneState.h"

namespace BallisticApp {

class StateDecelerating : public IDroneState {
public:
  DroneStateType execute(float& speed, float& direction, float desiredDir, const DroneConfig& cfg) override;
  DroneStateType getType() const override;
};

}  // namespace BallisticApp

#pragma once

#include "BallisticApp/interfaces/IDroneState.h"

namespace BallisticApp {

class StateMoving : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(DroneContext& ctx, float dt) override;
  DroneState getType() const override;
};

}  // namespace BallisticApp

#pragma once

#include "BallisticApp/DroneContext.h"
#include <cmath>

namespace BallisticApp {

class StateDecelerating : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(DroneContext& ctx, float dt) override;
  DroneState getType() const override;
};

}  // namespace BallisticApp
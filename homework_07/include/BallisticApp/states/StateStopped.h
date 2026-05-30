#pragma once

#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/DroneContext.h"
#include "BallisticApp/DroneState.h"

namespace BallisticApp {

class StateStopped : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(DroneContext& ctx, float dt) override;
  DroneState getType() const override;
};

}  // namespace BallisticApp

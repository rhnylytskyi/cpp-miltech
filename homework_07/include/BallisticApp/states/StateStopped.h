#pragma once

#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/MissionContext.h"
#include "BallisticApp/DroneState.h"
#include <memory>

namespace BallisticApp {

class StateStopped : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(MissionContext& ctx) override;
  DroneState getType() const override;
};

}  // namespace BallisticApp

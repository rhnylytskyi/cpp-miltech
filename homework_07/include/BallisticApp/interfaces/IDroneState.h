#pragma once

#include "BallisticApp/DroneState.h"
#include "BallisticApp/MissionContext.h"
#include <memory>

namespace BallisticApp {

class IDroneState {
public:
  virtual ~IDroneState() = default;
  virtual std::unique_ptr<IDroneState> execute(MissionContext& ctx) = 0;
  virtual DroneState getType() const = 0;
};

}  // namespace BallisticApp

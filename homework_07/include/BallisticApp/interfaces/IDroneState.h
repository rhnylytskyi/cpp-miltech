#pragma once

#include "BallisticApp/DroneState.h"

namespace BallisticApp {

struct MissionContext;

class IDroneState {
public:
  virtual ~IDroneState() = default;

  virtual DroneState execute(MissionContext& ctx) = 0;
  virtual DroneState getType() const = 0;
};

}  // namespace BallisticApp

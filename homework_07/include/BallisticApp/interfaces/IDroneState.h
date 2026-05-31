#pragma once

#include "BallisticApp/types/DroneStateType.h"

namespace BallisticApp {

struct MissionContext;

class IDroneState {
public:
  virtual ~IDroneState() = default;

  virtual DroneStateType execute(MissionContext& ctx) = 0;
  virtual DroneStateType getType() const = 0;
};

}  // namespace BallisticApp

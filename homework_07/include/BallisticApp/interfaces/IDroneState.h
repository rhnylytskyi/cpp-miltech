#pragma once

#include "BallisticApp/DroneStateType.h"

namespace BallisticApp {

struct MissionContext;

class IDroneState {
public:
  virtual ~IDroneState() = default;

  virtual DroneStateType execute(MissionContext& ctx) = 0;
};

}  // namespace BallisticApp

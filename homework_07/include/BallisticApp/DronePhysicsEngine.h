#pragma once

#include "BallisticApp/MissionContext.h"

namespace BallisticApp {

class DronePhysicsEngine {
public:
  DronePhysicsEngine() = default;

  void update(MissionContext& ctx) const;
};

}  // namespace BallisticApp

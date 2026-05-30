#pragma once

#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/DroneContext.h"
#include <memory>

namespace BallisticApp {

class DronePhysicsEngine {
public:
  DronePhysicsEngine(const DroneConfig& config);

  void update(DroneContext& ctx, std::unique_ptr<IDroneState>& currentState, const Coord& firePoint, float dt) const;

private:
  DroneConfig m_config;
};

}  // namespace BallisticApp

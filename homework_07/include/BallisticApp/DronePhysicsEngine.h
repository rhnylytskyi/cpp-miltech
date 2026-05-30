#pragma once

#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/MissionContext.h"
#include <memory>

namespace BallisticApp {

class DronePhysicsEngine {
public:
  DronePhysicsEngine(const DroneConfig& config);

  void update(MissionContext& ctx, std::unique_ptr<IDroneState>& currentState, const Coord& firePoint) const;

private:
  DroneConfig m_config;
};

}  // namespace BallisticApp

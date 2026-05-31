#pragma once

#include "BallisticApp/MissionContext.h"
#include "BallisticApp/types/DroneConfig.h"

namespace BallisticApp {

class DronePhysicsEngine {
public:
  DronePhysicsEngine(const DroneConfig& config);

  void update(MissionContext& ctx) const;

private:
  const DroneConfig& m_config;
};

}  // namespace BallisticApp

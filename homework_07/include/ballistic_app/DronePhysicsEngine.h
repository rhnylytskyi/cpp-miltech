#pragma once

#include "ballistic_app/Types.h"

namespace BallisticApp {
class DronePhysicsEngine {
public:
  DronePhysicsEngine(const DroneConfig& config);
  void update(DronePhysicsState& drone, const Coord& firePoint, float dt, float& outDeltaPath) const;

private:
  DroneConfig m_config;
};
}  // namespace BallisticApp

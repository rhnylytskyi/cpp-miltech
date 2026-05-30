#pragma once

#include "BallisticApp/Coord.h"
#include "BallisticApp/utils/MathUtils.h"
#include "BallisticApp/DroneState.h"
#include "BallisticApp/DroneConfig.h"

namespace BallisticApp {

struct MissionContext {
  Coord pos{0.0f, 0.0f};
  float speed{0.0f};
  float direction{0.0f};
  float desiredDir{0.0f};
  float lastDeltaPath{0.0f};
  const DroneConfig& cfg;

  inline bool isTargetCaptured(DroneState state, const Coord& firePoint) const
  {
    return (state == DroneState::MOVING) && (Math::length(this->pos - firePoint) <= this->cfg.hitRadius * 0.25f);
  }
};

}  // namespace BallisticApp

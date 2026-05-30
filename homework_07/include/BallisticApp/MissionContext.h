#pragma once

#include "BallisticApp/Coord.h"
#include "BallisticApp/utils/MathUtils.h"
#include "BallisticApp/DroneState.h"
#include "BallisticApp/DroneConfig.h"

namespace BallisticApp {

class IDroneState;

struct MissionContext {
  Coord pos{0.0f, 0.0f};
  float speed{0.0f};
  float direction{0.0f};
  float desiredDir{0.0f};
  float lastDeltaPath{0.0f};
  const DroneConfig& cfg;

  IDroneState* currentState{nullptr};
  DroneState currentStateType{DroneState::STOPPED};
  Coord firePoint{0.0f, 0.0f};

  float currentTime{0.0f};
  float cachedFlightTime{0.0f};
  float cachedHDist{0.0f};

  inline bool isTargetCaptured() const
  {
    return (this->currentStateType == DroneState::MOVING) && (Math::length(this->pos - this->firePoint) <= this->cfg.hitRadius * 0.25f);
  }

  inline MissionContext clone() const { return *this; }
};

}  // namespace BallisticApp

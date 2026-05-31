#pragma once

#include "BallisticApp/types/Coord.h"
#include "BallisticApp/types/DroneStateType.h"
#include "BallisticApp/types/DroneConfig.h"

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
  DroneStateType currentStateType{DroneStateType::STOPPED};
  Coord firePoint{0.0f, 0.0f};

  float currentTime{0.0f};
  float flightTime{0.0f};
  float hDistance{0.0f};

  inline bool isTargetCaptured() const
  {
    return (this->currentStateType == DroneStateType::MOVING) && ((this->pos - this->firePoint).length() <= this->cfg.hitRadius * 0.25f);
  }

  inline MissionContext clone() const { return *this; }
};

}  // namespace BallisticApp

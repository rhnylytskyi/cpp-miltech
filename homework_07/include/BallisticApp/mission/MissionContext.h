#pragma once

#include "BallisticApp/types/Coord.h"
#include "BallisticApp/types/DroneStateType.h"
#include "BallisticApp/config/DroneConfig.h"
#include "BallisticApp/interfaces/IDroneState.h"

namespace BallisticApp {

struct MissionContext {
  Coord pos{0.0f, 0.0f};
  float speed{0.0f};
  float direction{0.0f};
  float desiredDir{0.0f};
  float lastDeltaPath{0.0f};
  const DroneConfig& cfg;

  IDroneState* currentState{nullptr};
  Coord firePoint{0.0f, 0.0f};

  float currentTime{0.0f};
  float flightTime{0.0f};
  float hDistance{0.0f};

  inline DroneStateType getCurrentStateType() const { return currentState ? currentState->getType() : DroneStateType::STOPPED; }

  inline bool isTargetCaptured() const
  {
    return (getCurrentStateType() == DroneStateType::MOVING) && ((this->pos - this->firePoint).length() <= this->cfg.hitRadius * 0.25f);
  }
};

}  // namespace BallisticApp

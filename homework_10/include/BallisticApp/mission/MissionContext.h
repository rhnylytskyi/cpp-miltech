#pragma once

#include "BallisticApp/types/Coord.h"
#include "BallisticApp/types/DroneStateType.h"
#include "BallisticApp/config/DroneConfig.h"
#include "BallisticApp/interfaces/IDroneState.h"

namespace BallisticApp {

/**
 * @brief Context structure holding the runtime flight and ballistic state for the drone state machine.
 */
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
  float deltaTime{0.1f};

  explicit MissionContext(const DroneConfig& droneCfg)
    : cfg(droneCfg)
  {
  }

  DroneStateType getCurrentStateType() const { return currentState ? currentState->getType() : DroneStateType::STOPPED; }

  /**
   * @brief Physically accurate weapon release check leveraging ballistic drop distance.
   */
  bool isTargetCaptured() const
  {
    if (getCurrentStateType() != DroneStateType::MOVING) {
      return false;
    }

    // Bomb flies forward along the drone's direction vector for a distance of hDistance
    Coord bombLanding = this->pos + Coord{std::cos(this->direction), std::sin(this->direction)} * this->hDistance;

    // Check the distance from the bomb's landing point to the target's geometric point (where the target should be)
    const float currentDistanceSq = (bombLanding - this->firePoint).lengthSquared();
    const float fullRadiusSq = this->cfg.hitRadius * this->cfg.hitRadius;

    return currentDistanceSq <= fullRadiusSq;
  }
};

}  // namespace BallisticApp

#pragma once

#include "BallisticApp/types/Coord.h"
#include "BallisticApp/types/DroneStateType.h"
#include "BallisticApp/config/DroneConfig.h"
#include "BallisticApp/interfaces/IDroneState.h"
#include <cmath>

namespace BallisticApp {

/**
 * @brief Target capture validation modes.
 */
enum class CaptureAlgorithm {
  CLASSIC_RADIUS,         // Hard micro-radius approach (0.25 of hit radius)
  DOT_PRODUCT_OVERSHOOT,  // Distance check combined with overshoot detection using dot product
  RAY_SEGMENT_EDGE,       // Distance from target to the closest point on the current frame trajectory segment
  PERPENDICULAR_CENTER    // Сross-section intersection within the current frame interval
};

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

  CaptureAlgorithm activeAlgo = CaptureAlgorithm::PERPENDICULAR_CENTER;

  DroneStateType getCurrentStateType() const { return currentState ? currentState->getType() : DroneStateType::STOPPED; }

  /**
   * @brief Universal target capture check with dynamic algorithm switching.
   */
  bool isTargetCaptured() const
  {
    if (getCurrentStateType() != DroneStateType::MOVING) {
      return false;
    }

    // Common vector calculations using the Coord structure
    const Coord toFirePoint = this->firePoint - this->pos;
    const float currentDistanceSq = toFirePoint.lengthSquared();
    const float fullRadiusSq = this->cfg.hitRadius * this->cfg.hitRadius;

    // Heading direction vector calculated from the drone's current rotation angle
    const Coord headingDir = {std::cos(this->direction), std::sin(this->direction)};

    switch (this->activeAlgo) {
      case CaptureAlgorithm::CLASSIC_RADIUS: {
        /**
         * @brief CLASSIC_RADIUS: Validates if the drone has entered a strict,
         * downscaled core zone around the target center.
         */
        const float captureRadius = this->cfg.hitRadius * 0.25f;
        return currentDistanceSq <= (captureRadius * captureRadius);
      }

      case CaptureAlgorithm::DOT_PRODUCT_OVERSHOOT: {
        /**
         * @brief DOT_PRODUCT_OVERSHOOT: Combines the core micro-radius check with an
         * overshoot validator. Uses the dot product to see if the target is now behind the drone.
         */
        const float captureRadius = this->cfg.hitRadius * 0.25f;
        if (currentDistanceSq <= captureRadius * captureRadius) {
          return true;
        }

        // Projecting the vector to target onto the forward movement vector.
        // Negative result means the target has been passed over (is behind the drone).
        const float projectionAhead = toFirePoint.dot(headingDir);

        return (projectionAhead < 0.0f && currentDistanceSq <= this->lastDeltaPath * this->lastDeltaPath);
      }

      case CaptureAlgorithm::RAY_SEGMENT_EDGE: {
        /**
         * @brief RAY_SEGMENT_EDGE: Finds the closest point to the target on the
         * current frame's movement path segment, clamping the results to the edge boundaries.
         */
        if (this->lastDeltaPath <= 0.0001f) {
          return currentDistanceSq <= fullRadiusSq;
        }

        const Coord prevPos = this->pos - (headingDir * this->lastDeltaPath);
        const Coord toTargetFromStart = this->firePoint - prevPos;

        // Scalar projection to find how far along the trajectory segment the target sits
        float t = toTargetFromStart.dot(headingDir);

        // Clamping the projection to the current frame's trajectory boundaries
        if (t < 0.0f)
          t = 0.0f;
        if (t > this->lastDeltaPath)
          t = this->lastDeltaPath;

        // Finding the exact closest point and calculating squared distance to target
        const Coord closestPoint = prevPos + (headingDir * t);
        return (this->firePoint - closestPoint).lengthSquared() <= fullRadiusSq;
      }

      case CaptureAlgorithm::PERPENDICULAR_CENTER: {
        /**
         * @brief PERPENDICULAR_CENTER: Checks if the drone crossed the target's
         * perpendicular center-line specifically during the duration of this frame.
         */
        const Coord prevPos = this->pos - (headingDir * this->lastDeltaPath);
        const Coord toTargetFromStart = this->firePoint - prevPos;

        // Dot product determines the perpendicular intersection point 't' on the trajectory line
        const float t = toTargetFromStart.dot(headingDir);

        // Verify if the crossing happened within the current frame interval
        if (t >= 0.0f && t <= this->lastDeltaPath) {
          const Coord closestPoint = prevPos + (headingDir * t);
          if ((this->firePoint - closestPoint).lengthSquared() <= fullRadiusSq) {
            return true;
          }
        }

        // Fallback emergency trigger for extremely close proximities
        return currentDistanceSq <= 0.01f;
      }
    }

    return false;
  }
};

}  // namespace BallisticApp

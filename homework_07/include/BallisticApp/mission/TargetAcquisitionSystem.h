#pragma once

#include "BallisticApp/types/Coord.h"
#include "BallisticApp/config/DroneConfig.h"
#include "BallisticApp/ballistics/TargetExtrapolator.h"
#include "BallisticApp/ballistics/FireControlComputer.h"
#include "BallisticApp/utils/Logger.h"
#include "BallisticApp/mission/MissionContext.h"
#include <vector>
#include <limits>
#include <cmath>
#include <memory>
#include <utility>

namespace BallisticApp {

/**
 * @brief Target Acquisition System (TAS). Handles radar visibility zones,
 * filters distant targets, maintains target lock states, and computes mission timelines.
 */
class TargetAcquisitionSystem {
public:
  explicit TargetAcquisitionSystem(int targetCount)
    : m_knownVisibleTargets(targetCount, false)
  {
  }

  // === SECURITY BOUNDARIES ===
  TargetAcquisitionSystem(const TargetAcquisitionSystem&) = delete;
  TargetAcquisitionSystem& operator=(const TargetAcquisitionSystem&) = delete;

  /**
   * @brief Scans for potential objectives using long-range radar and secures the optimal firing asset.
   */
  std::pair<int, FireSolution> acquireBestTarget(const MissionContext& ctx,
                                                 const std::unique_ptr<TargetExtrapolator>& extrapolator,
                                                 const std::unique_ptr<FireControlComputer>& fireControl)
  {
    FireSolution bestSolution;
    bestSolution.isSuccess = false;
    bestSolution.time = std::numeric_limits<float>::max();
    int bestTarget = -1;

    // 1. Target Lock Persistence Logic
    if (m_enableTargetLock && m_lockedTargetId != -1) {
      FireSolution lockedSol = fireControl->calculateSolution(ctx, m_lockedTargetId);
      if (lockedSol.isSuccess) {
        bestSolution = lockedSol;
        bestTarget = m_lockedTargetId;
      }
    }

    // 2. Dynamic Radar Scanning Cycle
    if (bestTarget == -1) {
      m_lockedTargetId = -1;

      for (size_t tId = 0; tId < m_knownVisibleTargets.size(); ++tId) {
        const Coord instantaneousTargetPos = extrapolator->extrapolate(tId, ctx.currentTime, 0.0f);

        // Calculate ground 2D distance squared using pure 2D vector mathematics
        const float distance2DSq = (instantaneousTargetPos - ctx.pos).lengthSquared();

        // Convert to true 3D line-of-sight distance using Pythagoras theorem with altitude
        const float altitudeSq = ctx.cfg.altitude * ctx.cfg.altitude;
        const float distanceToTarget3D = std::sqrt(distance2DSq + altitudeSq);

        // LONG-RANGE RADAR: The scanner tracks any target within 10 kilometers.
        // This ensures the drone detects faraway objectives right from the start.
        const bool isCurrentlyVisible = (distanceToTarget3D <= 10000.0f);

        // Track visibility status changes to control logger outputs (prevents console spam)
        if (isCurrentlyVisible != m_knownVisibleTargets[tId]) {
          m_knownVisibleTargets[tId] = isCurrentlyVisible;
          if (isCurrentlyVisible) {
            APP_LOG_MOD("TAS Radar", "{:.<15} ID={:0>2} RANGE={:.2f}m", "OBJ_ACQUIRED", tId, distanceToTarget3D);
          }
          else {
            APP_LOG_MOD("TAS Radar", "{:.<15} ID={:0>2}", "OBJ_LOST", tId);
          }
        }

        if (!isCurrentlyVisible) {
          continue;
        }

        FireSolution sol = fireControl->calculateSolution(ctx, tId);
        if (sol.isSuccess && sol.time < bestSolution.time) {
          bestSolution = sol;
          bestTarget = static_cast<int>(tId);
        }
      }

      // Automatically secure weapon-system lock on the best acquired target
      if (m_enableTargetLock && bestTarget != -1) {
        m_lockedTargetId = bestTarget;
        logTargetLockDetails(ctx, bestSolution);
      }
    }

    return {bestTarget, bestSolution};
  }

  void reset(int targetCount) noexcept
  {
    m_lockedTargetId = -1;
    m_knownVisibleTargets.clear();
    m_knownVisibleTargets.resize(targetCount, false);
  }

  void setTargetLockEnabled(bool enabled) noexcept { m_enableTargetLock = enabled; }
  bool isTargetLockEnabled() const noexcept { return m_enableTargetLock; }
  int getLockedTargetId() const noexcept { return m_lockedTargetId; }

private:
  int m_lockedTargetId{-1};
  bool m_enableTargetLock{false};
  std::vector<bool> m_knownVisibleTargets;

  /**
   * @brief Physically-grounded dynamic tracking radius using drone travel capabilities and fall speeds.
   * Scaled defensively based on altitude percentage thresholds.
   */
  float calculateDynamicDetectionRadius(const MissionContext& ctx, const DroneConfig& cfg) const noexcept
  {
    const float ammoForwardPath = ctx.hDistance;
    const float droneTravelPath = ctx.speed * ctx.flightTime * 1.5f;
    const float hitZone = cfg.hitRadius;
    const float calculatedRadius = ammoForwardPath + droneTravelPath + hitZone;

    // Adaptive threshold: Altitude + 30% of current operational height as a stable cone buffer
    const float currentAltitude = std::abs(cfg.altitude);
    const float absoluteMinRadius = currentAltitude + (currentAltitude * 0.3f) + 20.0f;

    return calculatedRadius < absoluteMinRadius ? absoluteMinRadius : calculatedRadius;
  }

  /**
   * @brief Logs TTI metrics upon establishing a solid track lock.
   */
  void logTargetLockDetails(const MissionContext& ctx, const FireSolution& sol) const
  {
    float droneTimeToDrop = 0.0f;

    // Fallback protection: If drone speed is too low, we assume maximum attack speed
    // from config to prevent division by zero or static locking bugs.
    const float effectiveSpeed = (ctx.speed > 1e-4f) ? ctx.speed : ctx.cfg.attackSpeed;

    if (effectiveSpeed > 1e-4f) {
      // Calculate clean horizontal distance from current position directly to the future firePoint
      const float distanceToDropPoint = (sol.firePoint - ctx.pos).length();
      droneTimeToDrop = distanceToDropPoint / effectiveSpeed;
    }

    const float realBombFallTime = ctx.flightTime;
    const float totalTimeToImpact = droneTimeToDrop + realBombFallTime;

    APP_LOG_MOD("TAS Lock",
                "Secured track on target #{}. Timeline -> Drone approach: {:.2f}s | Bomb fall: {:.2f}s | Total TTI: {:.2f}s",
                m_lockedTargetId,
                droneTimeToDrop,
                realBombFallTime,
                totalTimeToImpact);
  }
};

}  // namespace BallisticApp

#pragma once

#include "ballistic_app/DronePhysicsEngine.h"
#include "ballistic_app/TargetPredictor.h"
#include "ballistic_app/Types.h"

namespace BallisticApp {

class MissionPlanner {
public:
  MissionPlanner(DronePhysicsEngine& physicsEngine, TargetPredictor& predictor, const DroneConfig& config);

  MissionPlanner(const MissionPlanner&) = delete;
  MissionPlanner& operator=(const MissionPlanner&) = delete;

  MissionPlanner(MissionPlanner&&) = delete;
  MissionPlanner& operator=(MissionPlanner&&) = delete;

  float predictTimeAndPos(const DronePhysicsState& currentDrone,
                          float currentTime,
                          float cachedFlightTime,
                          float cachedHDist,
                          int targetIdx,
                          Coord& outFirePoint,
                          Coord& outPredictedTarget) const;

private:
  DronePhysicsEngine& m_physicsEngine;
  TargetPredictor& m_predictor;
  const DroneConfig& m_config;
};

}  // namespace BallisticApp

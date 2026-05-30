#pragma once

#include "BallisticApp/DronePhysicsEngine.h"
#include "BallisticApp/TargetPredictor.h"
#include "BallisticApp/DroneConfig.h"

namespace BallisticApp {

class MissionPlanner {
public:
  MissionPlanner(DronePhysicsEngine& physicsEngine, TargetPredictor& predictor, const DroneConfig& config);

  MissionPlanner(const MissionPlanner&) = delete;
  MissionPlanner& operator=(const MissionPlanner&) = delete;

  MissionPlanner(MissionPlanner&&) = delete;
  MissionPlanner& operator=(MissionPlanner&&) = delete;

  float predictTimeAndPos(const DroneContext& currentDroneCtx,
                          DroneState currentStateType,
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

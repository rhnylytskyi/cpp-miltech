#pragma once

#include "ballistic_app/DronePhysicsEngine.h"
#include "ballistic_app/TargetPredictor.h"

namespace BallisticApp {
class MissionPlanner {
public:
  MissionPlanner(DronePhysicsEngine* physics, TargetPredictor* predictor, const DroneConfig& config);

  float predictTimeAndPos(const DronePhysicsState& currentDrone,
                          float currentTime,
                          float cachedFlightTime,
                          float cachedHDist,
                          int targetIdx,
                          Coord& outFirePoint,
                          Coord& outPredictedTarget) const;

private:
  DronePhysicsEngine* m_physicsEngine;
  TargetPredictor* m_predictor;
  DroneConfig m_config;
};
}  // namespace BallisticApp

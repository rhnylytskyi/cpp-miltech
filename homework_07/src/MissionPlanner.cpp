#include "ballistic_app/MissionPlanner.h"
#include "ballistic_app/utils/MathUtils.h"

namespace BallisticApp {
MissionPlanner::MissionPlanner(DronePhysicsEngine* physics, TargetPredictor* predictor, const DroneConfig& config)
  : m_physics(physics)
  , m_predictor(predictor)
  , m_config(config)
{
}

float MissionPlanner::predictTimeAndPos(const DronePhysicsState& currentDrone,
                                        float currentTime,
                                        float cachedFlightTime,
                                        float cachedHDist,
                                        int targetIdx,
                                        Coord& outFirePoint,
                                        Coord& outPredictedTarget) const
{
  DronePhysicsState vDrone = currentDrone;
  float vTime = currentTime;
  float elapsedPredictionTime = 0.0f;
  const float MAX_PREDICT_TIME = 30.0f;
  float dt = m_config.simTimeStep;

  while (elapsedPredictionTime < MAX_PREDICT_TIME) {
    if (m_predictor) {
      outPredictedTarget = m_predictor->extrapolate(targetIdx, vTime, cachedFlightTime);
    }

    Coord delta = outPredictedTarget - vDrone.pos;
    outFirePoint = outPredictedTarget - Math::normalize(delta) * cachedHDist;

    if (vDrone.state == DroneState::MOVING && Math::length(vDrone.pos - outFirePoint) <= m_config.hitRadius * 0.25f) {
      return elapsedPredictionTime;
    }

    float deltaPath = 0.0f;
    if (m_physics) {
      m_physics->update(vDrone, outFirePoint, dt, deltaPath);
    }

    vTime += dt;
    elapsedPredictionTime += dt;
  }
  
  return MAX_PREDICT_TIME;
}
}  // namespace BallisticApp

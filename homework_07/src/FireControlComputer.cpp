#include "BallisticApp/FireControlComputer.h"
#include "BallisticApp/DronePhysicsEngine.h"
#include "BallisticApp/TargetExtrapolator.h"
#include "BallisticApp/MissionContext.h"
#include <cmath>
#include <limits>

namespace BallisticApp {

FireControlComputer::FireControlComputer(DronePhysicsEngine& physicsEngine, TargetExtrapolator& extrapolator, float simTimeStep)
  : m_physicsEngine(physicsEngine)
  , m_extrapolator(extrapolator)
  , m_simTimeStep(simTimeStep)
{
}

FireSolution FireControlComputer::calculateSolution(const MissionContext& currentMissionCtx, int targetIdx) const
{
  FireSolution solution;
  solution.targetId = targetIdx;
  solution.time = std::numeric_limits<float>::max();
  solution.isSuccess = false;

  if (!currentMissionCtx.currentState) {
    return solution;
  }

  MissionContext vMissionCtx = currentMissionCtx;

  const float dt = m_simTimeStep;
  float elapsedPredictionTime = 0.0f;

  while (elapsedPredictionTime < MAX_PREDICT_TIME) {
    solution.predictedTarget = m_extrapolator.extrapolate(targetIdx, vMissionCtx.currentTime, vMissionCtx.flightTime);

    Coord delta = solution.predictedTarget - vMissionCtx.pos;
    if (delta.length() > 1e-4f) {
      vMissionCtx.firePoint = solution.predictedTarget - delta.normalize() * vMissionCtx.hDistance;
    }
    else {
      vMissionCtx.firePoint = solution.predictedTarget;
    }

    vMissionCtx.currentTime += dt;
    elapsedPredictionTime += dt;

    m_physicsEngine.update(vMissionCtx);

    if (vMissionCtx.isTargetCaptured()) {
      solution.time = elapsedPredictionTime;
      solution.firePoint = vMissionCtx.firePoint;
      solution.isSuccess = true;
      return solution;
    }
  }

  return solution;
}

}  // namespace BallisticApp

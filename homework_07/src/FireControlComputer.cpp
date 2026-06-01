#include "BallisticApp/FireControlComputer.h"
#include "BallisticApp/TargetExtrapolator.h"
#include "BallisticApp/MissionContext.h"
#include "BallisticApp/navigation/DroneAutopilot.h"
#include <cmath>
#include <limits>

namespace BallisticApp {

namespace {
constexpr float MAX_PREDICT_TIME = 30.0f;
}  // namespace

FireControlComputer::FireControlComputer(TargetExtrapolator& extrapolator, const DroneAutopilot& autopilot, float simTimeStep)
  : m_extrapolator(extrapolator)
  , m_autopilot(autopilot)
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

  MissionContext virtualCtx = currentMissionCtx;

  const float dt = m_simTimeStep;
  float elapsedPredictionTime = 0.0f;

  while (elapsedPredictionTime < MAX_PREDICT_TIME) {
    solution.predictedTarget = m_extrapolator.extrapolate(targetIdx, virtualCtx.currentTime, virtualCtx.flightTime);

    Coord delta = solution.predictedTarget - virtualCtx.pos;
    if (delta.length() > 1e-4f) {
      virtualCtx.firePoint = solution.predictedTarget - delta.normalize() * virtualCtx.hDistance;
    }
    else {
      virtualCtx.firePoint = solution.predictedTarget;
    }

    virtualCtx.currentTime += dt;
    elapsedPredictionTime += dt;

    // Update the autopilot with the virtual mission context
    m_autopilot.update(virtualCtx);

    if (virtualCtx.isTargetCaptured()) {
      solution.time = elapsedPredictionTime;
      solution.firePoint = virtualCtx.firePoint;
      solution.isSuccess = true;
      return solution;
    }
  }

  return solution;
}

}  // namespace BallisticApp

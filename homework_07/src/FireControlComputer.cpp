#include "BallisticApp/FireControlComputer.h"
#include "BallisticApp/DronePhysicsEngine.h"
#include "BallisticApp/TargetPredictor.h"
#include "BallisticApp/MissionContext.h"
#include <cmath>

namespace BallisticApp {

FireControlComputer::FireControlComputer(DronePhysicsEngine& physicsEngine, TargetPredictor& predictor, const DroneConfig& config)
  : m_physicsEngine(physicsEngine)
  , m_predictor(predictor)
  , m_config(config)
{
}

FireSolution FireControlComputer::calculateSolution(const MissionContext& currentMissionCtx, int targetIdx) const
{
  FireSolution solution;
  float dt = m_config.simTimeStep;

  if (!currentMissionCtx.currentState) {
    return solution;  // Повернеться isSuccess = false, time = 30.0f
  }

  MissionContext vMissionCtx = currentMissionCtx.clone();
  float elapsedPredictionTime = 0.0f;

  while (elapsedPredictionTime < MAX_PREDICT_TIME) {
    // Рахуємо позицію цілі
    solution.predictedTarget = m_predictor.extrapolate(targetIdx, vMissionCtx.currentTime, vMissionCtx.flightTime);

    // Рахуємо точку скидання
    Coord delta = solution.predictedTarget - vMissionCtx.pos;
    vMissionCtx.firePoint = solution.predictedTarget - delta.normalize() * vMissionCtx.hDistance;

    m_physicsEngine.update(vMissionCtx);

    if (vMissionCtx.isTargetCaptured()) {
      solution.time = elapsedPredictionTime + dt;
      solution.firePoint = vMissionCtx.firePoint;
      solution.isSuccess = true;
      return solution;  // Рішення знайдено!
    }

    vMissionCtx.currentTime += dt;
    elapsedPredictionTime += dt;
  }

  return solution;  // Повертаємо дефолтне невдале рішення, якщо не встигли наздогнати
}

}  // namespace BallisticApp

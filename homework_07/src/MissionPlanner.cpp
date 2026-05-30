#include "BallisticApp/MissionPlanner.h"
#include "BallisticApp/DronePhysicsEngine.h"
#include "BallisticApp/TargetPredictor.h"
#include "BallisticApp/states/StateStopped.h"
#include "BallisticApp/states/StateAccelerating.h"
#include "BallisticApp/states/StateDecelerating.h"
#include "BallisticApp/states/StateTurning.h"
#include "BallisticApp/states/StateMoving.h"
#include "BallisticApp/utils/MathUtils.h"
#include <cmath>

namespace BallisticApp {

MissionPlanner::MissionPlanner(DronePhysicsEngine& physicsEngine, TargetPredictor& predictor, const DroneConfig& config)
  : m_physicsEngine(physicsEngine)
  , m_predictor(predictor)
  , m_config(config)
{
}

std::unique_ptr<IDroneState> createStateFromType(DroneState type)
{
  switch (type) {
    case DroneState::STOPPED:
      return std::make_unique<StateStopped>();
    case DroneState::ACCELERATING:
      return std::make_unique<StateAccelerating>();
    case DroneState::DECELERATING:
      return std::make_unique<StateDecelerating>();
    case DroneState::TURNING:
      return std::make_unique<StateTurning>();
    case DroneState::MOVING:
      return std::make_unique<StateMoving>();
  }
  return std::make_unique<StateStopped>();
}

/**
 * Прогнозує час досягнення цілі та позицію скидання бомби для заданого стану дрона та цілі.
 */
float MissionPlanner::predictTimeAndPos(const DroneContext& currentDroneCtx,
                                        DroneState currentStateType,
                                        float currentTime,
                                        float cachedFlightTime,
                                        float cachedHDist,
                                        int targetIdx,
                                        Coord& outFirePoint,
                                        Coord& outPredictedTarget) const
{
  // Створюємо копію контексту та стану для віртуальної симуляції вперед у часі
  DroneContext vDroneCtx = currentDroneCtx;
  std::unique_ptr<IDroneState> vState = createStateFromType(currentStateType);

  float vTime = currentTime;
  float elapsedPredictionTime = 0.0f;

  // Максимальний час прогнозу (наприклад, 30 секунд), щоб уникнути нескінченних циклів
  const float MAX_PREDICT_TIME = 30.0f;
  // Крок інтеграції для прогнозу (такий самий, як у симуляції)
  float dt = m_config.simTimeStep;

  while (elapsedPredictionTime < MAX_PREDICT_TIME) {
    // Екстраполюємо, де буде ЦІЛЬ у цей віртуальний момент часу + час падіння бомби
    outPredictedTarget = m_predictor.extrapolate(targetIdx, vTime, cachedFlightTime);

    // Рахуємо, де має бути точка СКИДАННЯ для цієї позиції цілі
    Coord delta = outPredictedTarget - vDroneCtx.pos;
    outFirePoint = outPredictedTarget - Math::normalize(delta) * cachedHDist;

    // Оновлюємо віртуальний контекст дрона на основі фізики та поточного стану
    m_physicsEngine.update(vDroneCtx, vState, outFirePoint, dt);

    // Перевіряємо умову перехоплення (чи долетів віртуальний дрон до точки скидання)
    if (vDroneCtx.isTargetCaptured(vState->getType(), outFirePoint)) {
      return elapsedPredictionTime + dt;
    }

    // Оновлюємо віртуальний час
    vTime += dt;
    elapsedPredictionTime += dt;
  }

  return MAX_PREDICT_TIME;  // Якщо ціль недосяжна за адекватний час
}

}  // namespace BallisticApp

#include "ballistic_app/MissionPlanner.h"
#include "ballistic_app/utils/MathUtils.h"

namespace BallisticApp {
MissionPlanner::MissionPlanner(DronePhysicsEngine* physics, TargetPredictor* predictor, const DroneConfig& config)
  : m_physics(physics)
  , m_predictor(predictor)
  , m_config(config)
{
}

/**
  * Прогнозує час досягнення цілі та позицію скидання бомби для заданого стану дрона та цілі.
  * Використовує віртуальну симуляцію руху дрона та цілі для визначення оптимальної точки скидання.
  * @param currentDrone - поточний фізичний стан дрона
  * @param currentTime - поточний час у симуляції
  * @param cachedFlightTime - попередньо обчислений час падіння бомби
  * @param cachedHDist - попередньо обчислена горизонтальна
  * @param targetIdx - індекс цілі для прогнозування
  * @param outFirePoint - вихідний параметр для отримання розрахованої точки скидання
  * @param outPredictedTarget - вихідний параметр для отримання прогнозованої позиції цілі
  * @return - прогнозований час досягнення цілі або MAX_PREDICT_TIME, якщо ціль недосяжна за адекватний час
 */
float MissionPlanner::predictTimeAndPos(const DronePhysicsState& currentDrone,
                                        float currentTime,
                                        float cachedFlightTime,
                                        float cachedHDist,
                                        int targetIdx,
                                        Coord& outFirePoint,
                                        Coord& outPredictedTarget) const
{
  // Копіюємо ПОТОЧНИЙ стан дрона для віртуального тесту
  DronePhysicsState vDrone = currentDrone;
  float vTime = currentTime;
  float elapsedPredictionTime = 0.0f;

  // Максимальний час прогнозу (наприклад, 30 секунд), щоб уникнути нескінченних циклів
  const float MAX_PREDICT_TIME = 30.0f;
  // Крок інтеграції для прогнозу (такий самий, як у симуляції)
  float dt = m_config.simTimeStep;

  while (elapsedPredictionTime < MAX_PREDICT_TIME) {
    if (m_predictor) {
      // Екстраполюємо, де буде ЦІЛЬ у цей віртуальний момент часу + час падіння бомби
      outPredictedTarget = m_predictor->extrapolate(targetIdx, vTime, cachedFlightTime);
    }

    // Рахуємо, де має бути точка СКИДАННЯ для цієї позиції цілі
    Coord delta = outPredictedTarget - vDrone.pos;
    outFirePoint = outPredictedTarget - Math::normalize(delta) * cachedHDist;

    // Перевіряємо умову перехоплення (чи долетів віртуальний дрон до точки скидання)
    if (vDrone.state == DroneState::MOVING && Math::length(vDrone.pos - outFirePoint) <= m_config.hitRadius * 0.25f) {
      return elapsedPredictionTime;
    }

    // Оновлюємо віртуальну позицію та час
    float deltaPath = 0.0f;
    if (m_physics) {
      m_physics->update(vDrone, outFirePoint, dt, deltaPath);
    }

    vTime += dt;
    elapsedPredictionTime += dt;
  }
  
  return MAX_PREDICT_TIME; // Якщо ціль недосяжна за адекватний час
}
}  // namespace BallisticApp

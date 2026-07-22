#include "BallisticApp/mission/FlightController.h"
#include <cmath>
#include <algorithm>
#include <numbers>

namespace BallisticApp {

float FlightController::clamp1(float v) const
{
  return std::max(-1.0f, std::min(1.0f, v));
}

dlink::Control FlightController::compute(
  const dlink::Telemetry& tel, float desiredDir, DroneStateType stateType, float turnThreshold, float attackSpeed) const
{
  float headingErr = desiredDir - tel.dir;

  // Використовуємо точні константи C++20 замість магічних чисел
  constexpr float kPi = std::numbers::pi_v<float>;
  constexpr float kTwoPi = 2.0f * kPi;

  while (headingErr > kPi)
    headingErr -= kTwoPi;
  while (headingErr < -kPi)
    headingErr += kTwoPi;

  // Пропорційне керування з насиченням
  float turnRate = clamp1(headingErr / m_turnSaturationRad);

  // 2. РОЗРАХУНОК ПРИСКОРЕННЯ (accel)
  float targetSpeed = tel.speed;  // fallback

  if (stateType == DroneStateType::MOVING || stateType == DroneStateType::ACCELERATING) {
    // Курс рівний — прагнемо до максимальної швидкості атаки, переданої з конфігу чекера
    float speedFrac = 1.0f;
    float absErr = std::fabs(headingErr);

    // Якщо кутова помилка велика, плавно знижуємо цільову швидкість для кращого маневру
    if (absErr > turnThreshold) {
      float span = std::max(1e-3f, m_sharpTurnRad - turnThreshold);
      float t = clamp1((absErr - turnThreshold) / span);
      t = std::max(0.0f, t);
      speedFrac = 1.0f - t * (1.0f - m_minSpeedFrac);
    }

    // ТЕПЕР ТУТ ПРАВИЛЬНА МАТЕМАТИКА: розганяємось до attackSpeed, зважаючи на кут!
    targetSpeed = attackSpeed * speedFrac;
  }
  else if (stateType == DroneStateType::DECELERATING) {
    // Смикаємо гальма до нуля
    targetSpeed = 0.0f;
  }
  else {
    // TURNING або STOPPED — стоїмо на місці
    targetSpeed = 0.0f;
  }

  // Обчислюємо похибку швидкості
  float speedErr = targetSpeed - tel.speed;

  // Якщо ми розганяємось, а швидкість замала — даємо повний вперед
  if ((stateType == DroneStateType::ACCELERATING || stateType == DroneStateType::MOVING) && speedErr > 0.05f) {
    return dlink::Control{1.0f, turnRate};
  }
  // Якщо гальмуємо — повний назад
  if (stateType == DroneStateType::DECELERATING) {
    return dlink::Control{-1.0f, turnRate};
  }

  // Пропорційне утримання швидкості у зоні m_accelBand
  float band = std::max(0.1f, tel.speed * m_accelBand);
  float accel = clamp1(speedErr / band);

  return dlink::Control{accel, turnRate};
}

}  // namespace BallisticApp

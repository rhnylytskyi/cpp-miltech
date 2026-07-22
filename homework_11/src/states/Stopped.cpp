// src/states/StateStopped.cpp
#include "BallisticApp/states/Stopped.h"
#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/utils/MathUtils.h"
#include <cmath>

namespace BallisticApp::states {

DroneStateType StateStopped::execute(float& speed, float& direction, float desiredDir, const DroneConfig& cfg)
{
  // Скидаємо швидкість до 0, оскільки дрон зупинений
  speed = 0.0f;

  // Рахуємо кутову помилку за вашим фірмовим методом
  const float deltaAngle = Math::normalizeAngle(desiredDir - direction);

  // Якщо відхилення більше за поріг — переходимо в режим розвороту
  if (std::fabs(deltaAngle) > cfg.turnThreshold) {
    return DroneStateType::TURNING;
  }

  // Якщо курс рівний — починаємо розгін
  return DroneStateType::ACCELERATING;
}

DroneStateType StateStopped::getType() const
{
  return DroneStateType::STOPPED;
}

}  // namespace BallisticApp::states

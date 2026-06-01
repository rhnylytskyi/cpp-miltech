#pragma once

#include "BallisticApp/TargetExtrapolator.h"
#include "BallisticApp/types/Coord.h"

namespace BallisticApp {

class DronePhysicsEngine;
class TargetExtrapolator;
struct MissionContext;

// Структура комплексного балістичного рішення
struct FireSolution {
  int targetId{-1};
  bool isSuccess{false};        // Чи знайдено математичне рішення
  float time{0.0f};             // Час до відкриття вогню/скидання (задається обчислювачем)
  Coord firePoint{0.0f, 0.0f};  // Оптимальна точка скидання
  Coord predictedTarget{0.0f, 0.0f};  // Прогнозовані координати цілі в момент влучання
};

class FireControlComputer {
public:
  static constexpr float MAX_PREDICT_TIME = 30.0f;

  FireControlComputer(DronePhysicsEngine& physicsEngine, TargetExtrapolator& extrapolator, float simTimeStep);

  /**
   * @brief Розраховує балістичне рішення для атаки заданої цілі.
   *
   * @param currentMissionCtx Поточний контекст місії.
   * @param targetIdx Індекс цілі для екстраполяції її руху.
   * @return FireSolution Структура з повним результатом розрахунку.
   */
  FireSolution calculateSolution(const MissionContext& currentMissionCtx, int targetIdx) const;

private:
  DronePhysicsEngine& m_physicsEngine;
  TargetExtrapolator& m_extrapolator;
  float m_simTimeStep;
};

}  // namespace BallisticApp

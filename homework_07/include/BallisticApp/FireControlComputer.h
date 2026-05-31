#pragma once

#include "BallisticApp/types/DroneConfig.h"
#include "BallisticApp/types/Coord.h"

namespace BallisticApp {

class DronePhysicsEngine;
class TargetPredictor;
struct MissionContext;

// Структура комплексного балістичного рішення
struct FireSolution {
  float time{30.0f};                  // Час до відкриття вогню/скидання
  Coord firePoint{0.0f, 0.0f};        // Оптимальна точка скидання
  Coord predictedTarget{0.0f, 0.0f};  // Прогнозовані координати цілі в момент влучання
  bool isSuccess{false};              // Чи знайдено математичне рішення
};

class FireControlComputer {
public:
  static constexpr float MAX_PREDICT_TIME = 30.0f;

  FireControlComputer(DronePhysicsEngine& physicsEngine, TargetPredictor& predictor, const DroneConfig& config);

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
  TargetPredictor& m_predictor;
  const DroneConfig& m_config;
};

}  // namespace BallisticApp

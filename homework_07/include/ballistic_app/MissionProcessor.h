#pragma once

#include "ballistic_app/interfaces/IConfigLoader.h"
#include "ballistic_app/interfaces/ITargetProvider.h"
#include "ballistic_app/interfaces/IBallisticSolver.h"
#include "ballistic_app/interfaces/ISimulationExporter.h"
#include "ballistic_app/Types.h"

namespace BallisticApp {

class MissionProcessor {
public:
  static constexpr int MAX_STEPS = 10000;

  MissionProcessor(IConfigLoader* loader, ITargetProvider* provider, IBallisticSolver* solver, ISimulationExporter* exporter);
  ~MissionProcessor();

  void init(const char* configSource, const char* ammoSource);
  bool hasNext();
  SimStep step();
  void reset();
  void changeSolver(IBallisticSolver* s);

  int getTotalSteps() const;
  const SimStep* getStepsHistory() const;

  /**
   * @brief Запускає повний цикл симуляції до її завершення та автоматично експортує результати.
   */
  void run();

private:
  // Поля класу (Вказівники на інтерфейси)
  IConfigLoader* m_loader;
  ITargetProvider* m_provider;
  IBallisticSolver* m_solver;
  ISimulationExporter* m_exporter;

  // Дані конфігурації та пам'ять для кроків
  DroneConfig m_config;
  AmmoParams m_ammo;
  SimStep* m_steps;

  // Поточний стан реальної симуляції
  Coord m_dronePos;
  float m_direction;
  float m_speed;
  DroneState m_state;
  float m_currentTime;
  int m_totalSteps;
  bool m_isMissionFinished;

  // Кешовані значення балістичного розрахунку
  float m_cachedFlightTime = 0.0f;
  float m_cachedHDist = 0.0f;

  // Допоміжна структура для передачі та розрахунку стану фізики
  struct DronePhysicsState {
    Coord pos;
    float speed;
    float direction;
    DroneState state;
  };

  // Методи інтерполяції та екстраполяції цілей
  Coord interpolateTarget(int targetIdx, float t);
  Coord extrapolateTarget(int targetIdx, float time, float dt);

  // Методи для прогнозування та фізики
  float predictTimeAndPos(int targetIdx, Coord& outFirePoint, Coord& outPredictedTarget);
  void updateDronePhysics(DronePhysicsState& drone, const Coord& firePoint, float dt, float& outDeltaPath);
};

}  // namespace BallisticApp

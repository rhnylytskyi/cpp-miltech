#pragma once

#include "ballistic_app/io/interfaces/IConfigLoader.hpp"
#include "ballistic_app/io/interfaces/ISimulationExporter.hpp"
#include "ballistic_app/providers/ITargetProvider.hpp"
#include "ballistic_app/solvers/IBallisticSolver.hpp"
#include "ballistic_app/dto/DroneConfig.hpp"
#include "ballistic_app/dto/AmmoParams.hpp"
#include "ballistic_app/dto/SimStep.hpp"
#include "ballistic_app/dto/Coord.hpp"
#include "ballistic_app/dto/DroneState.hpp"

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
  DroneConfig config;
  AmmoParams ammo;
  SimStep* steps;

  // Поточний стан реальної симуляції
  Coord dronePos;
  float direction;
  float speed;
  DroneState state;
  float currentTime;
  int totalSteps;
  bool isMissionFinished;

  // Кешовані значення балістичного розрахунку
  float cachedFlightTime = 0.0f;
  float cachedHDist = 0.0f;

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

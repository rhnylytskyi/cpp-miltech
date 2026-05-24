#pragma once

#include "ballistic_app/interfaces/IConfigLoader.h"
#include "ballistic_app/interfaces/ITargetProvider.h"
#include "ballistic_app/interfaces/IBallisticSolver.h"
#include "ballistic_app/interfaces/ISimulationExporter.h"
#include "ballistic_app/DronePhysicsEngine.h"
#include "ballistic_app/TargetPredictor.h"
#include "ballistic_app/MissionPlanner.h"

namespace BallisticApp {
class MissionProcessor {
public:
  static const int MAX_STEPS = 10000;

  MissionProcessor(IConfigLoader* loader, ITargetProvider* provider, IBallisticSolver* solver, ISimulationExporter* exporter);
  ~MissionProcessor();

  void init(const char* configSource, const char* ammoSource);
  bool hasNext();
  SimStep step();
  void run();
  void reset();
  void changeSolver(IBallisticSolver* solver);

  int getTotalSteps() const;
  const SimStep* getStepsHistory() const;

private:
  // Зовнішні залежності (інтерфейси)
  IConfigLoader* m_loader;
  ITargetProvider* m_provider;
  IBallisticSolver* m_solver;
  ISimulationExporter* m_exporter;

  DronePhysicsEngine* m_physicsEngine;
  TargetPredictor* m_targetPredictor;
  MissionPlanner* m_planner;

  DroneConfig m_config;
  AmmoParams m_ammo;
  SimStep* m_steps;

  Coord m_dronePos;
  float m_direction;
  float m_speed;
  DroneState m_state;

  float m_currentTime;
  int m_totalSteps;
  bool m_isMissionFinished;

  float m_cachedFlightTime;
  float m_cachedHDist;
};
}  // namespace BallisticApp

#pragma once

#include "ballistic_app/Types.h"
#include "ballistic_app/DronePhysicsEngine.h"
#include "ballistic_app/TargetPredictor.h"
#include "ballistic_app/MissionPlanner.h"
#include <string>
#include <vector>
#include <memory>

namespace BallisticApp {

class IConfigLoader;
class ITargetProvider;
class IBallisticSolver;
class ISimulationExporter;

class MissionProcessor {
public:
  static constexpr int MAX_STEPS = 10000;

  MissionProcessor(const std::string& targetsPath,
                   const std::string& simulationPath,
                   const std::string& configSource,
                   const std::string& ammoSource);
  ~MissionProcessor();

  MissionProcessor(const MissionProcessor&) = delete;
  MissionProcessor& operator=(const MissionProcessor&) = delete;

  bool hasNext();
  SimStep step();
  void run();
  void reset();
  void changeSolver(std::unique_ptr<IBallisticSolver> solver);

  int getTotalSteps() const;
  const std::vector<SimStep>& getStepsHistory() const;

private:
  struct TargetCandidate {
    int id;
    float time;
    Coord firePoint;
    Coord predictedTarget;
  };

  std::unique_ptr<IConfigLoader> m_loader;
  std::unique_ptr<ITargetProvider> m_provider;
  std::unique_ptr<IBallisticSolver> m_solver;
  std::unique_ptr<ISimulationExporter> m_exporter;

  DroneConfig m_config;
  AmmoParams m_ammo;

  DronePhysicsEngine m_physicsEngine;
  TargetPredictor m_targetPredictor;
  MissionPlanner m_planner;

  std::vector<SimStep> m_steps;
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

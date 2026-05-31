#pragma once

#include "BallisticApp/DroneConfig.h"
#include "BallisticApp/AmmoParams.h"
#include "BallisticApp/MissionContext.h"
#include "BallisticApp/DronePhysicsEngine.h"
#include "BallisticApp/TargetPredictor.h"
#include "BallisticApp/FireControlComputer.h"
#include "BallisticApp/SimStep.h"
#include <filesystem>
#include <vector>
#include <memory>

namespace BallisticApp {

class IConfigLoader;
class ITargetProvider;
class IBallisticSolver;
class ISimulationExporter;

struct TargetCandidate {
  int id{0};
  FireSolution solution;
};

class MissionProcessor {
public:
  static constexpr int MAX_STEPS = 10000;

  MissionProcessor(const std::filesystem::path& configSource,
                   const std::filesystem::path& targetsPath,
                   const std::filesystem::path& ammoSource,
                   const std::filesystem::path& simulationPath);
  ~MissionProcessor();

  void run();
  SimStep step();
  void reset();
  bool hasNext();
  void changeSolver(std::unique_ptr<IBallisticSolver> solver);
  int getTotalSteps() const;
  const std::vector<SimStep>& getStepsHistory() const;

private:
  void updateBallisticCache();

  std::unique_ptr<IConfigLoader> m_loader;
  std::unique_ptr<ITargetProvider> m_provider;
  std::unique_ptr<IBallisticSolver> m_solver;
  std::unique_ptr<ISimulationExporter> m_exporter;

  DroneConfig m_config;
  AmmoParams m_ammo;

  std::unique_ptr<DronePhysicsEngine> m_physicsEngine;
  std::unique_ptr<TargetPredictor> m_targetPredictor;
  std::unique_ptr<FireControlComputer> m_fireControl;

  MissionContext m_missionCtx;
  float m_currentTime;
  int m_totalSteps;
  bool m_isMissionFinished;
  std::vector<SimStep> m_steps;
  std::vector<TargetCandidate> m_candidates;
};

}  // namespace BallisticApp

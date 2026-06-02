#pragma once

#include "BallisticApp/ballistics/TargetExtrapolator.h"
#include "BallisticApp/ballistics/FireControlComputer.h"
#include "BallisticApp/navigation/DroneAutopilot.h"
#include "BallisticApp/config/DroneConfig.h"
#include "BallisticApp/config/AmmoParams.h"
#include "BallisticApp/exporters/SimStep.h"
#include "BallisticApp/mission/MissionContext.h"
#include "BallisticApp/mission/TargetAcquisitionSystem.h"
#include <filesystem>
#include <vector>
#include <memory>

namespace BallisticApp {

class IConfigLoader;
class ITargetProvider;
class IBallisticSolver;
class ISimulationExporter;
class FireControlComputer;

class MissionProcessor {
public:
  MissionProcessor(const std::filesystem::path& configSource,
                   const std::filesystem::path& targetsPath,
                   const std::filesystem::path& ammoSource,
                   const std::filesystem::path& simulationPath,
                   bool targetLockEnabled = false);
  ~MissionProcessor();

  bool hasNext();
  SimStep step();
  void run();
  void reset();

  void changeSolver(std::unique_ptr<IBallisticSolver> solver);
  int getTotalSteps() const;
  const std::vector<SimStep>& getStepsHistory() const;

  private:
  std::unique_ptr<IConfigLoader> m_loader;
  std::unique_ptr<ITargetProvider> m_provider;
  std::unique_ptr<IBallisticSolver> m_solver;
  std::unique_ptr<ISimulationExporter> m_exporter;

  DroneConfig m_config;
  AmmoParams m_ammo;

  DroneAutopilot m_autopilot;
  TargetExtrapolator m_extrapolator;
  std::unique_ptr<FireControlComputer> m_fireControl;

  MissionContext m_missionCtx;
  TargetAcquisitionSystem m_tas;

  float m_currentTime{0.0f};
  int m_totalSteps{0};
  bool m_isMissionFinished{false};
  std::vector<SimStep> m_steps;

  void updateBallisticCache();
};

}  // namespace BallisticApp

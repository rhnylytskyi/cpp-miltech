#pragma once

#include "BallisticApp/ballistics/TargetExtrapolator.h"
#include "BallisticApp/ballistics/FireControlComputer.h"
#include "BallisticApp/navigation/DroneAutopilot.h"
#include "BallisticApp/config/DroneConfig.h"
#include "BallisticApp/config/AmmoParams.h"
#include "BallisticApp/exporters/SimStep.h"
#include "BallisticApp/mission/MissionContext.h"
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

  DroneAutopilot m_autopilot;
  TargetExtrapolator m_extrapolator;
  std::unique_ptr<FireControlComputer> m_fireControl;

  MissionContext m_missionCtx;
  float m_currentTime;
  int m_totalSteps;
  bool m_isMissionFinished;
  std::vector<SimStep> m_steps;
};

}  // namespace BallisticApp

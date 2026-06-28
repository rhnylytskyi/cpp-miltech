#pragma once

#include "BallisticApp/navigation/DroneAutopilot.h"
#include "BallisticApp/ballistics/TargetExtrapolator.h"
#include "BallisticApp/mission/TargetAcquisitionSystem.h"
#include "BallisticApp/exporters/SimStep.h"
#include "BallisticApp/config/AmmoParams.h"
#include <vector>
#include <memory>
#include <filesystem>

namespace BallisticApp {

class IConfigLoader;
class ITargetProvider;
class IBallisticSolver;
class ISimulationExporter;
class FireControlComputer;

enum class SolverType : int; 

struct LaunchParams {
  std::filesystem::path configPath;
  std::filesystem::path targetsPath;
  std::filesystem::path ammoPath;
  std::filesystem::path simulationPath;
  std::filesystem::path tablePath;
  SolverType solverType;
  bool enableTargetLock = false;
};

class MissionProcessor {
public:
  explicit MissionProcessor(const LaunchParams& params);
  ~MissionProcessor();

  bool hasNext() const;
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

  // --- Фізичні параметри та модулі автопілота ---
  DroneConfig m_config;
  AmmoParams m_ammo;
  DroneAutopilot m_autopilot;
  std::unique_ptr<TargetExtrapolator> m_extrapolator;
  std::unique_ptr<FireControlComputer> m_fireControl;

  // --- Контекст місії та радар ---
  MissionContext m_missionCtx;
  TargetAcquisitionSystem m_tas;

  // --- Стан симуляції ---
  float m_currentTime{0.0f};
  int m_totalSteps{0};
  bool m_isMissionFinished{false};
  std::vector<SimStep> m_steps;

  void updateBallisticCache();
};

}  // namespace BallisticApp

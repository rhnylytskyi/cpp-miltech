#pragma once

#include "BallisticApp/config/DroneConfig.h"
#include "BallisticApp/config/AmmoParams.h"
#include "BallisticApp/interfaces/IConfigLoader.h"
#include "BallisticApp/interfaces/IBallisticSolver.h"
#include "BallisticApp/interfaces/ISimulationExporter.h"
#include "BallisticApp/providers/ThreadSafeTargetProvider.h"
#include "BallisticApp/navigation/DronePhysics.h"
#include "BallisticApp/exporters/SimStep.h"
#include <memory>
#include <vector>
#include <mutex>
#include <atomic>

namespace BallisticApp {

class AppArguments;

class MissionProcessor {
public:
  MissionProcessor(const AppArguments& appArgs, ThreadSafeTargetProvider& provider, DronePhysics& physics);
  ~MissionProcessor();

  MissionProcessor(const MissionProcessor&) = delete;
  MissionProcessor& operator=(const MissionProcessor&) = delete;

  void run();
  void reset();

  bool isThreadReady() const;
  void start();
  void stop();

  bool hasNext() const;
  SimStep step();

  void changeSolver(std::unique_ptr<IBallisticSolver> solver);
  int getTotalSteps() const;
  std::vector<SimStep> getStepsHistory() const;

private:
  void updateBallisticCache();

  ThreadSafeTargetProvider& m_provider;
  DronePhysics& m_physics;

  std::unique_ptr<IConfigLoader> m_loader;
  std::unique_ptr<IBallisticSolver> m_solver;
  std::unique_ptr<ISimulationExporter> m_exporter;

  // Внутрішні параметри та кеш балістики
  DroneConfig m_config;
  AmmoParams m_ammo;
  float m_cachedFlightTime{0.0f};
  float m_cachedHDistance{0.0f};

  // Логіка вибору та фіксації цілей
  int m_lockedTargetId{-1};
  bool m_enableTargetLockCLI{false};
  float m_prevHitDistance{1e9f};

  // Синхронізація кадрів та історія кроків симуляції
  uint64_t m_telemetrySeqNum{0};
  int m_totalSteps{0};
  bool m_isMissionFinished{false};
  std::vector<SimStep> m_steps;

  // Керування життєвим циклом потоку місії
  mutable std::mutex m_stepsMutex;
  std::atomic<bool> m_isReady{false};
  std::atomic<bool> m_isStarted{false};
  std::atomic<bool> m_stopRequested{false};
};

}  // namespace BallisticApp

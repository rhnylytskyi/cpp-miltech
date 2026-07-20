#pragma once

#include "BallisticApp/config/DroneConfig.h"
#include "BallisticApp/navigation/DroneCommand.h"
#include "BallisticApp/navigation/DroneTelemetry.h"
#include "BallisticApp/types/DroneStateType.h"
#include "BallisticApp/navigation/ThreadSafeQueue.h"
#include <mutex>
#include <atomic>
#include <condition_variable>

namespace BallisticApp {

struct Coord;

class DronePhysics {
public:
  DronePhysics() = default;
  ~DronePhysics();

  DronePhysics(const DronePhysics&) = delete;
  DronePhysics& operator=(const DronePhysics&) = delete;

  void configure(const DroneConfig& config);
  void reset(const Coord& startPos, float initialDir);
  void run();

  bool isThreadReady() const;
  void start();
  void stop();

  // Methods for inter-thread communication
  void postCommand(const DroneCommand& cmd);
  DroneTelemetry getTelemetry() const;

  // STIFF BARRIER: Blocking wait for new telemetry from MissionProcessor
  DroneTelemetry waitTelemetry(uint64_t& lastSeq);

private:
  void integratePhysicsStep(const DroneCommand& cmd);
  void publishTelemetryLocked();

  // Physical parameters of the drone
  DroneConfig m_config;
  Coord m_pos{0.0f, 0.0f};
  float m_direction{0.0f};
  float m_speed{0.0f};
  DroneStateType m_currentState{DroneStateType::STOPPED};

  // Deterministic virtual time
  float m_timeSecSinceStart{0.0f};
  float m_lastDeltaPath{0.0f};

  // Thread-safe queue for commands from the mission planner
  ThreadSafeQueue<DroneCommand> m_commandQueue;

  // Thread-safe cache for telemetry and synchronization barrier
  mutable std::mutex m_mutex;
  mutable std::condition_variable m_cv;
  DroneTelemetry m_telemetryCache;
  uint64_t m_telemetrySeq{0};

  // Lifecycle management for the mission thread
  std::atomic<bool> m_isReady{false};
  std::atomic<bool> m_isStarted{false};
  std::atomic<bool> m_stopRequested{false};

  // Atomic for confirming frame consumption by the mission
  std::atomic<uint64_t> m_consumedSeq{0};
};

}  // namespace BallisticApp

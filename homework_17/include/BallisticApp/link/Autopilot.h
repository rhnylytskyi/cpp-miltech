#pragma once

#include "BallisticApp/DroneStateType.h"
#include "BallisticApp/link/FlightController.h"
#include "BallisticApp/link/GpioPins.h"
#include "BallisticApp/link/UartLink.h"
#include "BallisticApp/link/UartTargetProvider.h"
#include "drone_link.h"
#include <memory>
#include <mutex>
#include <atomic>

// Forward declaration of the new network module to avoid circular includes
namespace BallisticApp::net {
class MavlinkTelemetry;
}

namespace BallisticApp {
class IBallisticSolver;
struct Coord;
}  // namespace BallisticApp

namespace BallisticApp::mission {

class Autopilot {
public:
  static bool handshake(link::UartLink& link, link::GpioPins& gpio, dlink::AmmoCfg& outAmmo, dlink::DroneCfg& outCfg, int timeoutMs = 5000);

  Autopilot(link::UartLink& link,
            link::GpioPins& gpio,
            std::unique_ptr<IBallisticSolver> solver,
            const dlink::AmmoCfg& ammo,
            const dlink::DroneCfg& cfg,
            std::shared_ptr<net::MavlinkTelemetry> mavlink = nullptr);  // Added MAVLink pointer injection

  ~Autopilot() noexcept;

  Autopilot(const Autopilot&) = delete;
  Autopilot& operator=(const Autopilot&) = delete;

  void runIO();       // Thread 1: Continuous UART stream fetching
  void runMission();  // Thread 2: Ballistics and FSM scheduling loop

  void start() noexcept;
  void stop() noexcept;
  [[nodiscard]] bool isThreadReady() const noexcept;
  [[nodiscard]] bool dropped() const noexcept { return m_dropped.load(); }
  [[nodiscard]] bool isFinished() const noexcept;

private:
  void onTelemetryReceived(const dlink::Telemetry& tel);
  void executeMissionStep(const dlink::Telemetry& tel);

  [[nodiscard]] Coord predictTargetIntercept(int targetIdx, const Coord& dronePos, float ballisticTime, float ballisticDist) const noexcept;

  link::UartLink& m_link;
  link::GpioPins& m_gpio;
  std::unique_ptr<IBallisticSolver> m_solver;

  dlink::AmmoCfg m_ammo;
  dlink::DroneCfg m_cfg;

  link::UartTargetProvider m_targets;
  FlightController m_flight;
  std::shared_ptr<net::MavlinkTelemetry> m_mavlink;  // Core network telemetry module pointer

  std::atomic<float> m_lastTSec{0.0f};
  std::atomic<float> m_missionStartSec{-1.0f};
  std::atomic<bool> m_dropped{false};
  std::atomic<bool> m_missionFinished{false};

  std::atomic<bool> m_isIoReady{false};
  std::atomic<bool> m_isMissionReady{false};
  std::atomic<bool> m_isStarted{false};
  std::atomic<bool> m_stopRequested{false};

  mutable std::mutex m_telemetryMutex;
  dlink::Telemetry m_latestTelemetry{};
  bool m_hasTelemetry{false};

  int m_currentTarget{-1};
  float m_currentTargetTime{1e9f};
  float m_prevHitDist{1e9f};
  Coord m_dropPoint{0.0f, 0.0f};
  DroneStateType m_currentFsmState{DroneStateType::STOPPED};

  std::mutex m_uartWriteMutex;
  uint32_t m_simTimeMs{0};  // Keep track of relative simulation timestamps for MAVLink clocks

  static constexpr float kMaxMissionTimeSec = 180.0f;
  static constexpr int kPredictionIterations = 8;
  static constexpr int kDropPulseDurationMs = 80;
};

}  // namespace BallisticApp::mission

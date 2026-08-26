#pragma once

#include "BallisticApp/DroneStateType.h"
#include "BallisticApp/control/FlightController.h"
#include "BallisticApp/exporters/SimStep.h"
#include "BallisticApp/link/GpioPins.h"
#include "BallisticApp/link/UartLink.h"
#include "BallisticApp/link/UartTargetProvider.h"
#include "drone_link.h"
#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

namespace BallisticApp {
class MavlinkTelemetry;
class IBallisticSolver;
struct Coord;
}  // namespace BallisticApp

namespace BallisticApp {

class Autopilot {
public:
  static bool handshake(UartLink& link, GpioPins& gpio, dlink::AmmoCfg& outAmmo, dlink::DroneCfg& outCfg, int timeoutMs = 5000);

  Autopilot(UartLink& link,
            GpioPins& gpio,
            std::unique_ptr<IBallisticSolver> solver,
            const dlink::AmmoCfg& ammo,
            const dlink::DroneCfg& cfg,
            std::shared_ptr<MavlinkTelemetry> mavlink = nullptr);

  ~Autopilot() noexcept;

  Autopilot(const Autopilot&) = delete;
  Autopilot& operator=(const Autopilot&) = delete;

  void runIO();
  void runMission();

  void start() noexcept;
  void stop() noexcept;
  [[nodiscard]] bool isThreadReady() const noexcept;
  [[nodiscard]] bool dropped() const noexcept { return m_dropped.load(); }
  [[nodiscard]] bool isFinished() const noexcept;

  [[nodiscard]] const std::vector<SimStep>& getSimulationSteps() const noexcept { return m_simSteps; }

private:
  void onTelemetryReceived(const dlink::Telemetry& tel);
  void executeMissionStep(const dlink::Telemetry& tel);

  [[nodiscard]] Coord predictTargetIntercept(int targetIdx, const Coord& dronePos, float ballisticTime, float ballisticDist) const noexcept;

  UartLink& m_link;
  GpioPins& m_gpio;
  std::unique_ptr<IBallisticSolver> m_solver;

  dlink::AmmoCfg m_ammo;
  dlink::DroneCfg m_cfg;

  UartTargetProvider m_targets;
  FlightController m_flight;
  std::shared_ptr<MavlinkTelemetry> m_mavlink;

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
  std::vector<SimStep> m_simSteps;

  std::mutex m_uartWriteMutex;
  uint32_t m_simTimeMs{0};

  static constexpr float kMaxMissionTimeSec = 180.0f;
  static constexpr int kPredictionIterations = 8;
  static constexpr int kDropPulseDurationMs = 80;
};

}  // namespace BallisticApp

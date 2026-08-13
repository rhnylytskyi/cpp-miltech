#pragma once

#include "BallisticApp/DroneStateType.h"
#include "BallisticApp/sys/UartLink.h"
#include "BallisticApp/sys/GpioPins.h"
#include "BallisticApp/sys/UartTargetProvider.h"
#include "BallisticApp/sys/MavlinkManager.h"
#include "BallisticApp/mission/FlightController.h"
#include "drone_link.h"
#include <memory>
#include <string>

namespace BallisticApp {
class IBallisticSolver;
struct Coord;
}  // namespace BallisticApp

namespace BallisticApp::mission {

class Autopilot {
public:
  static bool handshake(sys::UartLink& link, sys::GpioPins& gpio, dlink::AmmoCfg& outAmmo, dlink::DroneCfg& outCfg, int timeoutMs = 5000);

  Autopilot(sys::UartLink& link,
            sys::GpioPins& gpio,
            std::unique_ptr<IBallisticSolver> solver,
            const dlink::AmmoCfg& ammo,
            const dlink::DroneCfg& cfg,
            const std::string& mavHost = "127.0.0.1",
            uint16_t mavPort = 14550);

  ~Autopilot() noexcept = default;

  Autopilot(const Autopilot&) = delete;
  Autopilot& operator=(const Autopilot&) = delete;

  [[nodiscard]] bool step();
  [[nodiscard]] bool dropped() const noexcept { return m_dropped; }

private:
  void onTelemetry(const dlink::Telemetry& tel);
  [[nodiscard]] Coord predictTargetIntercept(int targetIdx, const Coord& dronePos, float ballisticTime, float ballisticDist) const noexcept;

  sys::UartLink& m_link;
  sys::GpioPins& m_gpio;
  std::unique_ptr<IBallisticSolver> m_solver;

  dlink::AmmoCfg m_ammo;
  dlink::DroneCfg m_cfg;

  sys::UartTargetProvider m_targets;
  FlightController m_flight;

  float m_lastTSec{0.0f};
  float m_missionStartSec{-1.0f};
  bool m_dropped{false};

  int m_currentTarget{-1};
  float m_currentTargetTime{1e9f};
  float m_prevHitDist{1e9f};
  Coord m_dropPoint{0.0f, 0.0f};

  DroneStateType m_currentFsmState{DroneStateType::STOPPED};

  sys::MavlinkManager m_mavlink;

  static constexpr float kMaxMissionTimeSec = 180.0f;
  static constexpr int kPredictionIterations = 8;
  static constexpr int kDropPulseDurationMs = 80;
};

}  // namespace BallisticApp::mission

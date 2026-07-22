#ifndef BALLISTICAPP_MISSION_AUTOPILOT_H
#define BALLISTICAPP_MISSION_AUTOPILOT_H

#include "BallisticApp/drivers/UartLink.h"
#include "BallisticApp/drivers/GpioPins.h"
#include "BallisticApp/drivers/UartTargetProvider.h"
#include "BallisticApp/mission/FlightController.h"
#include "BallisticApp/interfaces/IBallisticSolver.h"
#include "BallisticApp/types/Coord.h"
#include <memory>

namespace BallisticApp::mission {

class Autopilot {
public:
  static bool handshake(sys::UartLink& link, sys::GpioPins& gpio, dlink::AmmoCfg& outAmmo, dlink::DroneCfg& outCfg, int timeoutMs = 2000);

  Autopilot(sys::UartLink& link,
            sys::GpioPins& gpio,
            std::unique_ptr<IBallisticSolver> solver,
            const dlink::AmmoCfg& ammo,
            const dlink::DroneCfg& cfg);

  bool step();
  bool dropped() const { return m_dropped; }

private:
  void onTelemetry(const dlink::Telemetry& tel);

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

  static constexpr float kMaxMissionTimeSec = 22.0f;
};

}  // namespace BallisticApp::mission

#endif  // BALLISTICAPP_MISSION_AUTOPILOT_H

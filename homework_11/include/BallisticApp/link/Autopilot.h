#ifndef BALLISTICAPP_LINK_AUTOPILOT_H
#define BALLISTICAPP_LINK_AUTOPILOT_H

#include "BallisticApp/link/UartLink.h"
#include "BallisticApp/link/GpioPins.h"
#include "BallisticApp/link/UartTargetProvider.h"
#include "BallisticApp/link/FlightController.h"
#include "BallisticApp/interfaces/IBallisticSolver.h"
#include "BallisticApp/types/Coord.h"
#include <memory>

namespace BallisticApp {

class Autopilot {
public:
  // Статичний метод для фази рукостискання (Handshake)
  static bool handshake(UartLink& link, GpioPins& gpio, dlink::AmmoCfg& outAmmo, dlink::DroneCfg& outCfg, int timeoutMs = 2000);

  // Конструктор приймає ініціалізовані лінки та солвер
  Autopilot(
    UartLink& link, GpioPins& gpio, std::unique_ptr<IBallisticSolver> solver, const dlink::AmmoCfg& ammo, const dlink::DroneCfg& cfg);

  // Метод одного кроку польотного циклу
  bool step();

  // Прапорець для перевірки, чи відбувся скид
  bool dropped() const { return dropped_; }

private:
  void onTelemetry(const dlink::Telemetry& tel);

  UartLink& link_;
  GpioPins& gpio_;
  std::unique_ptr<IBallisticSolver> solver_;

  dlink::AmmoCfg ammo_;
  dlink::DroneCfg cfg_;

  UartTargetProvider targets_;
  FlightController flight_;

  float lastTSec_{0.0f};
  float missionStartSec_{-1.0f};
  bool dropped_{false};

  int currentTarget_{-1};
  float currentTargetTime_{1e9f};
  float prevHitDist_{1e9f};
  Coord dropPoint_{0.0f, 0.0f};

  static constexpr float kMaxMissionTimeSec = 22.0f;
};

}  // namespace BallisticApp

#endif  // BALLISTICAPP_LINK_AUTOPILOT_H
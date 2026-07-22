#pragma once
#include "drone_link.h"
#include "BallisticApp/interfaces/IDroneState.h"

namespace BallisticApp {

class FlightController {
private:
  const float m_turnSaturationRad = 0.4f;
  const float m_sharpTurnRad = 0.6f;
  const float m_minSpeedFrac = 0.2f;
  const float m_accelBand = 0.2f;

  float clamp1(float v) const;

public:
  FlightController() = default;

  dlink::Control compute(
    const dlink::Telemetry& tel, float desiredDir, DroneStateType stateType, float turnThreshold, float attackSpeed) const;
};

}  // namespace BallisticApp

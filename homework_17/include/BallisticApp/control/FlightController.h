#pragma once

#include "drone_link.h"
#include <cstdint>

namespace BallisticApp {
enum class DroneStateType : uint8_t;
}

namespace BallisticApp {

class FlightController {
public:
  FlightController() = default;
  ~FlightController() noexcept = default;

  FlightController(const FlightController&) = default;
  FlightController& operator=(const FlightController&) = default;

  [[nodiscard]] dlink::Control compute(
    const dlink::Telemetry& tel, float desiredDir, DroneStateType stateType, float turnThreshold, float attackSpeed) const noexcept;

private:
  static constexpr float m_turnSaturationRad = 0.4f;
  static constexpr float m_sharpTurnRad = 0.6f;
  static constexpr float m_minSpeedFrac = 0.2f;
  static constexpr float m_accelBand = 0.2f;
};

}  // namespace BallisticApp

#pragma once

#include "drone_link.h"
#include <cstdint>

namespace BallisticApp {

// Forward declaration of enum class to avoid including full state headers
enum class DroneStateType : uint8_t;

class FlightController {
private:
  // Flight dynamic limits optimized for the real-time control loop
  static constexpr float m_turnSaturationRad = 0.4f;
  static constexpr float m_sharpTurnRad = 0.6f;
  static constexpr float m_minSpeedFrac = 0.2f;
  static constexpr float m_accelBand = 0.2f;

public:
  FlightController() = default;
  ~FlightController() noexcept = default;

  /* Flight controllers are purely mathematical workers and do not own stateful streams */
  FlightController(const FlightController &) = default;
  FlightController &operator=(const FlightController &) = default;

  /**
   * @brief Translates high-level mission steering decisions into low-level UART control vectors.
   * @param tel Current drone telemetry frame containing speed and direction profiles.
   * @param desiredDir Calculated target interception bearing in radians.
   * @param stateType Current active state machine execution profile identifier.
   * @param turnThreshold Dynamic turn threshold configuration in radians.
   * @param attackSpeed Target forward velocity capability window in m/s.
   * @return A dlink::Control package containing normalized accel and turnRate outputs.
   */
  [[nodiscard]] dlink::Control compute(
    const dlink::Telemetry &tel, float desiredDir, DroneStateType stateType, float turnThreshold, float attackSpeed) const noexcept;
};

}  // namespace BallisticApp

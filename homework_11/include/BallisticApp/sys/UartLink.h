#pragma once

#include "BallisticApp/sys/UartPort.h"
#include "drone_link.h"
#include <functional>
#include <string>

namespace BallisticApp::sys {

/**
 * @brief Protocol orchestration layer bridging UartPort hardware IO and dlink::Parser frames.
 * Dispatches verified packages directly to mission control registers via registered callbacks.
 */
class UartLink {
public:
  using TelemetryCb = std::function<void(const dlink::Telemetry &)>;
  using TargetCb = std::function<void(const dlink::TargetPos &)>;
  using AmmoCb = std::function<void(const dlink::AmmoCfg &)>;
  using ConfigCb = std::function<void(const dlink::DroneCfg &)>;

  UartLink() = default;
  ~UartLink() noexcept = default;

  /* Strict resource linkage: protocol driver instances must remain unique */
  UartLink(const UartLink &) = delete;
  UartLink &operator=(const UartLink &) = delete;

  void open(const std::string &device) { m_port.open(device); }

  void onTelemetry(TelemetryCb cb) { m_onTelemetry = std::move(cb); }
  void onTarget(TargetCb cb) { m_onTarget = std::move(cb); }
  void onAmmo(AmmoCb cb) { m_onAmmo = std::move(cb); }
  void onConfig(ConfigCb cb) { m_onConfig = std::move(cb); }

  /**
   * @brief Flushes and completely resets the state machine variables of the internal frame parser.
   */
  void resetParser() noexcept;

  /**
   * @brief Non-blocking pump designed for real-time cyclic polling execution.
   * Consumes all serial line buffers and invokes relevant frame handlers.
   * @return Count of successfully parsed and verified data frames this tick.
   */
  [[nodiscard]] int pump() noexcept;

  /**
   * @brief Serializes and immediately transmits a flight steering control vector package.
   */
  void sendControl(float accel, float turnRate);

private:
  void dispatch(uint8_t type, const uint8_t *payload, uint8_t len) noexcept;

  UartPort m_port;
  dlink::Parser m_parser;

  TelemetryCb m_onTelemetry;
  TargetCb m_onTarget;
  AmmoCb m_onAmmo;
  ConfigCb m_onConfig;
};

}  // namespace BallisticApp::sys

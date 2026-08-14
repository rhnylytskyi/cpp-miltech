#pragma once

#include "BallisticApp/sys/UartPort.h"
#include "drone_link.h"
#include <functional>
#include <string>

namespace BallisticApp::sys {

/**
 * @brief Protocol driver bridging UartPort hardware IO and dlink::Parser frames.
 */
class UartLink {
public:
  using TelemetryCb = std::function<void(const dlink::Telemetry &)>;
  using TargetCb = std::function<void(const dlink::TargetPos &)>;
  using AmmoCb = std::function<void(const dlink::AmmoCfg &)>;
  using ConfigCb = std::function<void(const dlink::DroneCfg &)>;
  using ResultCb = std::function<void(const dlink::Result &)>;

  UartLink() = default;
  ~UartLink() noexcept = default;

  /* Unique runtime communication stream ownership */
  UartLink(const UartLink &) = delete;
  UartLink &operator=(const UartLink &) = delete;

  void open(const std::string &device) { m_port.open(device); }

  /* Frame callback registration */
  void onTelemetry(TelemetryCb cb) { m_onTelemetry = std::move(cb); }
  void onTarget(TargetCb cb) { m_onTarget = std::move(cb); }
  void onAmmo(AmmoCb cb) { m_onAmmo = std::move(cb); }
  void onConfig(ConfigCb cb) { m_onConfig = std::move(cb); }
  void onResult(ResultCb cb) { m_onResult = std::move(cb); }

  /* Stream processing handlers */
  void resetParser() noexcept;
  [[nodiscard]] int pump() noexcept;
  void sendControl(float accel, float turnRate);

private:
  void dispatch(uint8_t type, const uint8_t *payload, uint8_t len) noexcept;

  UartPort m_port;
  dlink::Parser m_parser;

  TelemetryCb m_onTelemetry;
  TargetCb m_onTarget;
  AmmoCb m_onAmmo;
  ConfigCb m_onConfig;
  ResultCb m_onResult;
};

}  // namespace BallisticApp::sys

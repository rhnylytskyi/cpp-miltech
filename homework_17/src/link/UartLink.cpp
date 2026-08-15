#include "BallisticApp/link/UartLink.h"
#include <cstring>

namespace BallisticApp {

void UartLink::resetParser() noexcept
{
  m_parser.st = dlink::Parser::S_M0;
  m_parser.idx = 0;
  m_parser.len = 0;
}

int UartLink::pump() noexcept
{
  uint8_t raw[256];
  int n = m_port.read(raw, sizeof(raw));
  int frames = 0;

  for (int i = 0; i < n; ++i) {
    uint8_t type = 0;
    uint8_t len = 0;
    uint8_t payload[260];
    if (m_parser.feed(raw[i], type, payload, len)) {
      dispatch(type, payload, len);
      ++frames;
    }
  }
  return frames;
}

void UartLink::dispatch(uint8_t type, const uint8_t* payload, uint8_t len) noexcept
{
  switch (type) {
    case dlink::PKT_TELEMETRY:
      if (m_onTelemetry && len == sizeof(dlink::Telemetry)) {
        dlink::Telemetry t;
        std::memcpy(&t, payload, sizeof(t));
        m_onTelemetry(t);
      }
      break;

    case dlink::PKT_TARGET:
      if (m_onTarget && len == sizeof(dlink::TargetPos)) {
        dlink::TargetPos p;
        std::memcpy(&p, payload, sizeof(p));
        m_onTarget(p);
      }
      break;

    case dlink::PKT_AMMO:
      if (m_onAmmo && len == sizeof(dlink::AmmoCfg)) {
        dlink::AmmoCfg a;
        std::memcpy(&a, payload, sizeof(a));
        m_onAmmo(a);
      }
      break;

    case dlink::PKT_CONFIG:
      if (m_onConfig && len == sizeof(dlink::DroneCfg)) {
        dlink::DroneCfg c;
        std::memcpy(&c, payload, sizeof(c));
        m_onConfig(c);
      }
      break;

    case dlink::PKT_RESULT:
      if (m_onResult && len == sizeof(dlink::Result)) {
        dlink::Result r;
        std::memcpy(&r, payload, sizeof(r));
        m_onResult(r);
      }
      break;

    default:
      break;
  }
}

void UartLink::sendControl(float accel, float turnRate)
{
  dlink::Control c{accel, turnRate};
  uint8_t out[64];
  size_t m = dlink::encode(dlink::PKT_CONTROL, &c, sizeof(c), out);
  m_port.write(out, m);
}

}  // namespace BallisticApp

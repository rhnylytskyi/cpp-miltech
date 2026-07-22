#include "BallisticApp/drivers/UartLink.h"
#include <chrono>
#include <cstring>
#include <thread>

using namespace dlink;

namespace BallisticApp::sys {

int UartLink::pump()
{
  uint8_t raw[256];
  int n = m_port.read(raw, sizeof raw);
  int frames = 0;

  for (int i = 0; i < n; ++i) {
    uint8_t type, len, payload[260];

    if (m_parser.feed(raw[i], type, payload, len)) {
      dispatch(type, payload, len);
      ++frames;
    }
  }

  return frames;
}

void UartLink::dispatch(uint8_t type, const uint8_t *payload, uint8_t len)
{
  switch (type) {
    case PKT_TELEMETRY:
      if (m_onTelemetry && len == sizeof(Telemetry)) {
        Telemetry t;
        std::memcpy(&t, payload, sizeof t);
        m_onTelemetry(t);
      }
      break;

    case PKT_TARGET:
      if (m_onTarget && len == sizeof(TargetPos)) {
        TargetPos p;
        std::memcpy(&p, payload, sizeof p);
        m_onTarget(p);
      }
      break;

    case PKT_AMMO:
      if (m_onAmmo && len == sizeof(AmmoCfg)) {
        AmmoCfg a;
        std::memcpy(&a, payload, sizeof a);
        m_onAmmo(a);
      }
      break;

    case PKT_CONFIG:
      if (m_onConfig && len == sizeof(DroneCfg)) {
        DroneCfg c;
        std::memcpy(&c, payload, sizeof c);
        m_onConfig(c);
      }
      break;

    default:
      break;  // невідомий/непотрібний студенту тип (напр. PKT_RESULT) — ігноруємо
  }
}

bool UartLink::waitFor(uint8_t type, int timeoutMs)
{
  // Ловимо потрібний тип кадру через тимчасовий callback-перехоплювач,
  // не чіпаючи постійні callback'и, встановлені користувачем.
  bool got = false;

  TelemetryCb prevTel = m_onTelemetry;
  TargetCb prevTgt = m_onTarget;
  AmmoCb prevAmmo = m_onAmmo;
  ConfigCb prevCfg = m_onConfig;

  if (type == PKT_TELEMETRY) {
    m_onTelemetry = [&got, prevTel](const Telemetry &t) {
      got = true;
      if (prevTel) {
        prevTel(t);
      }
    };
  }
  else if (type == PKT_TARGET) {
    m_onTarget = [&got, prevTgt](const TargetPos &p) {
      got = true;
      if (prevTgt) {
        prevTgt(p);
      }
    };
  }
  else if (type == PKT_AMMO) {
    m_onAmmo = [&got, prevAmmo](const AmmoCfg &a) {
      got = true;
      if (prevAmmo) {
        prevAmmo(a);
      }
    };
  }
  else if (type == PKT_CONFIG) {
    m_onConfig = [&got, prevCfg](const DroneCfg &c) {
      got = true;
      if (prevCfg) {
        prevCfg(c);
      }
    };
  }

  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);

  while (!got && std::chrono::steady_clock::now() < deadline) {
    pump();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  m_onTelemetry = prevTel;
  m_onTarget = prevTgt;
  m_onAmmo = prevAmmo;
  m_onConfig = prevCfg;

  return got;
}

void UartLink::sendControl(float accel, float turnRate)
{
  Control c{accel, turnRate};
  uint8_t out[64];
  size_t m = encode(PKT_CONTROL, &c, sizeof c, out);
  m_port.write(out, m);
}

}  // namespace BallisticApp::sys

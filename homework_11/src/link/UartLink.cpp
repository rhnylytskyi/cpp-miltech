#include "BallisticApp/link/UartLink.h"
#include <chrono>
#include <cstring>
#include <thread>

using namespace dlink;

int UartLink::pump()
{
  uint8_t raw[256];
  int n = port_.read(raw, sizeof raw);
  int frames = 0;

  for (int i = 0; i < n; ++i) {
    uint8_t type, len, payload[260];

    if (parser_.feed(raw[i], type, payload, len)) {
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
      if (onTelemetry_ && len == sizeof(Telemetry)) {
        Telemetry t;
        std::memcpy(&t, payload, sizeof t);
        onTelemetry_(t);
      }

      break;

    case PKT_TARGET:
      if (onTarget_ && len == sizeof(TargetPos)) {
        TargetPos p;
        std::memcpy(&p, payload, sizeof p);
        onTarget_(p);
      }

      break;

    case PKT_AMMO:
      if (onAmmo_ && len == sizeof(AmmoCfg)) {
        AmmoCfg a;
        std::memcpy(&a, payload, sizeof a);
        onAmmo_(a);
      }

      break;

    case PKT_CONFIG:
      if (onConfig_ && len == sizeof(DroneCfg)) {
        DroneCfg c;
        std::memcpy(&c, payload, sizeof c);
        onConfig_(c);
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

  TelemetryCb prevTel = onTelemetry_;
  TargetCb prevTgt = onTarget_;
  AmmoCb prevAmmo = onAmmo_;
  ConfigCb prevCfg = onConfig_;

  if (type == PKT_TELEMETRY) {
    onTelemetry_ = [&got, prevTel](const Telemetry &t) {
      got = true;

      if (prevTel) {
        prevTel(t);
      }
    };
  }
  else if (type == PKT_TARGET) {
    onTarget_ = [&got, prevTgt](const TargetPos &p) {
      got = true;

      if (prevTgt) {
        prevTgt(p);
      }
    };
  }
  else if (type == PKT_AMMO) {
    onAmmo_ = [&got, prevAmmo](const AmmoCfg &a) {
      got = true;

      if (prevAmmo) {
        prevAmmo(a);
      }
    };
  }
  else if (type == PKT_CONFIG) {
    onConfig_ = [&got, prevCfg](const DroneCfg &c) {
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

  onTelemetry_ = prevTel;
  onTarget_ = prevTgt;
  onAmmo_ = prevAmmo;
  onConfig_ = prevCfg;

  return got;
}

void UartLink::sendControl(float accel, float turnRate)
{
  Control c{accel, turnRate};
  uint8_t out[64];
  size_t m = encode(PKT_CONTROL, &c, sizeof c, out);
  port_.write(out, m);
}

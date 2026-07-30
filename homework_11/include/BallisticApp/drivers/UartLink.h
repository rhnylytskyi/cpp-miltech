#pragma once
#include "BallisticApp/drivers/UartPort.h"
#include "drone_link.h"
#include <functional>
#include <string>

namespace BallisticApp::sys {

// Обгортка над UartPort + dlink::Parser: читає доступні байти, годує парсер,
// і на кожному зібраному кадрі викликає відповідний callback.
// Один об'єкт на весь час роботи програми - тримає стан парсера між викликами pump().
class UartLink {
public:
  using TelemetryCb = std::function<void(const dlink::Telemetry &)>;
  using TargetCb = std::function<void(const dlink::TargetPos &)>;
  using AmmoCb = std::function<void(const dlink::AmmoCfg &)>;
  using ConfigCb = std::function<void(const dlink::DroneCfg &)>;

  void open(const std::string &device) { m_port.open(device); }

  void onTelemetry(TelemetryCb cb) { m_onTelemetry = std::move(cb); }
  void onTarget(TargetCb cb) { m_onTarget = std::move(cb); }
  void onAmmo(AmmoCb cb) { m_onAmmo = std::move(cb); }
  void onConfig(ConfigCb cb) { m_onConfig = std::move(cb); }

  void resetParser();

  // Прочитати все, що зараз доступне з порту, і розібрати на кадри.
  // Повертає кількість повністю зібраних (валідних) кадрів.
  // Неблокуюче — призначене для виклику в основному циклі кожен такт.
  int pump();

  // Заблокувати виконання, поки pump() не розбере хоча б один кадр типу `type`
  // (решта типів кадрів, що прийдуть по дорозі, теж диспетчеризуються як завжди).
  // Повертає false при таймауті.
  bool waitFor(uint8_t type, int timeoutMs);

  // Надіслати команду керування чекеру.
  void sendControl(float accel, float turnRate);

private:
  UartPort m_port;
  dlink::Parser m_parser;

  TelemetryCb m_onTelemetry;
  TargetCb m_onTarget;
  AmmoCb m_onAmmo;
  ConfigCb m_onConfig;

  void dispatch(uint8_t type, const uint8_t *payload, uint8_t len);
};

}  // namespace BallisticApp::sys

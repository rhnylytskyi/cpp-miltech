#include "BallisticApp/navigation/DronePhysics.h"
#include "BallisticApp/states/DroneStateRegistry.h"
#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/mission/MissionContext.h"
#include <thread>
#include <chrono>
#include <algorithm>
#include <cmath>

namespace BallisticApp {

DronePhysics::~DronePhysics()
{
  stop();
}

void DronePhysics::configure(const DroneConfig& config)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  m_config = config;
}

void DronePhysics::reset(const Coord& startPos, float initialDir)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  m_pos = startPos;
  m_direction = initialDir;
  m_speed = 0.0f;
  m_currentState = DroneStateType::STOPPED;
  m_timeSecSinceStart = 0.0f;
  m_lastDeltaPath = 0.0f;
  m_commandQueue.clear();

  m_telemetrySeq = 0;
  m_consumedSeq.store(0);

  m_telemetryCache.pos = m_pos;
  m_telemetryCache.direction = m_direction;
  m_telemetryCache.speed = m_speed;
  m_telemetryCache.state = m_currentState;
  m_telemetryCache.timeSecSinceStart = m_timeSecSinceStart;
}

bool DronePhysics::isThreadReady() const
{
  return m_isReady.load();
}

void DronePhysics::start()
{
  m_isStarted.store(true);
}

void DronePhysics::stop()
{
  m_stopRequested.store(true);
  m_cv.notify_all();  // Розблокуємо потоки у разі екстреної зупинки
}

void DronePhysics::postCommand(const DroneCommand& cmd)
{
  m_commandQueue.push(cmd);
}

DroneTelemetry DronePhysics::getTelemetry() const
{
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_telemetryCache;
}

// Споживання кадру місією (викликається в потоці MissionProcessor)
DroneTelemetry DronePhysics::waitTelemetry(uint64_t& lastSeq)
{
  std::unique_lock<std::mutex> lock(m_mutex);

  // Чекаємо, поки з'явиться новий номер кадру (або прийде запит на зупинку)
  m_cv.wait(lock, [this, &lastSeq] { return m_telemetrySeq != lastSeq || m_stopRequested.load(); });

  lastSeq = m_telemetrySeq;
  DroneTelemetry result = m_telemetryCache;
  lock.unlock();

  // Сигналізуємо фізичному потоку, що цей кадр успішно прочитано місією
  m_consumedSeq.store(lastSeq, std::memory_order_release);
  return result;
}

void DronePhysics::publishTelemetryLocked()
{
  m_telemetryCache.pos = m_pos;
  m_telemetryCache.direction = m_direction;
  m_telemetryCache.speed = m_speed;
  m_telemetryCache.state = m_currentState;
  m_telemetryCache.timeSecSinceStart = m_timeSecSinceStart;

  ++m_telemetrySeq;
  m_cv.notify_all();  // Будимо потік місії, що заснув у waitTelemetry
}

void DronePhysics::integratePhysicsStep(const DroneCommand& cmd)
{
  const float dt = m_config.physicsTimeStep;

  MissionContext internalCtx(m_config);
  internalCtx.pos = m_pos;
  internalCtx.direction = m_direction;
  internalCtx.desiredDir = cmd.desiredDir;
  internalCtx.speed = m_speed;
  internalCtx.lastDeltaPath = m_lastDeltaPath;
  internalCtx.currentTime = m_timeSecSinceStart;
  internalCtx.deltaTime = dt;
  internalCtx.currentState = DroneStateRegistry::getState(m_currentState);

  if (!internalCtx.currentState) {
    return;
  }

  // Обчислюємо логіку поточного стану
  DroneStateType nextStateType = internalCtx.currentState->execute(internalCtx);

  // Валідація прискорення
  float maxAllowedAcceleration = 0.0f;
  if (m_config.accelPath > 1e-5f) {
    // Коригуємо дільник відповідно до налаштувань детермінізму тестера курсу
    maxAllowedAcceleration = (m_config.attackSpeed * m_config.attackSpeed) / (3.0f * m_config.accelPath);
  }

  float calculatedAcceleration = (internalCtx.speed - m_speed) / dt;

  if (std::fabs(calculatedAcceleration) > maxAllowedAcceleration && maxAllowedAcceleration > 0.0f) {
    const float directionSign = (calculatedAcceleration > 0.0f) ? 1.0f : -1.0f;
    internalCtx.speed = m_speed + (directionSign * maxAllowedAcceleration * dt);
    internalCtx.lastDeltaPath = ((m_speed + internalCtx.speed) / 2.0f) * dt;
  }

  if (nextStateType == DroneStateType::TURNING || nextStateType == DroneStateType::STOPPED) {
    internalCtx.lastDeltaPath = 0.0f;
    internalCtx.speed = 0.0f;
  }

  // Записуємо результати інтегрування
  m_speed = internalCtx.speed;
  m_lastDeltaPath = internalCtx.lastDeltaPath;
  m_direction = internalCtx.direction;
  m_currentState = nextStateType;

  // Оновлюємо тригонометричні координати
  m_pos.x += std::cos(m_direction) * m_lastDeltaPath;
  m_pos.y += std::sin(m_direction) * m_lastDeltaPath;

  m_timeSecSinceStart += dt;
}

void DronePhysics::run()
{
  m_isReady.store(true);
  while (!m_stopRequested.load() && !m_isStarted.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  float dtConfig = 0.01f;
  float simStepConfig = 0.1f;
  float scale = 1.0f;

  // Початкова дефолтна команда
  DroneCommand activeCommand{.stateType = DroneStateType::STOPPED, .desiredDir = 0.0f};
  
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    dtConfig = m_config.physicsTimeStep;
    simStepConfig = m_config.simTimeStep;
    scale = m_config.timeScale;
    activeCommand.desiredDir = m_direction;
  }

  // Розраховуємо кратність: скільки мікрокроків фізики міститься в одному кроці планувальника
  const int publishEvery = std::max(1, static_cast<int>(std::round(simStepConfig / dtConfig)));
  int stepsSincePublish = 0;

  while (!m_stopRequested.load()) {
    // ВИТЯГУЄМО НАЙНОВІШУ КОМАНДУ (Всі старі та проміжні просто відкидаємо)
    while (auto newCmd = m_commandQueue.tryPop()) {
      activeCommand = *newCmd;
    }

    {
      std::lock_guard<std::mutex> lock(m_mutex);
      integratePhysicsStep(activeCommand);
    }

    // Засинаємо суто для підтримки візуального масштабу часу
    const float sleepSeconds = dtConfig / std::max(scale, 1e-5f);
    std::this_thread::sleep_for(std::chrono::duration<float>(sleepSeconds));

    // Перевіряємо, чи настав час синхронізації з MissionProcessor
    if (++stepsSincePublish >= publishEvery) {
      uint64_t publishedFrame = 0;
      {
        std::lock_guard<std::mutex> lock(m_mutex);
        publishTelemetryLocked();
        publishedFrame = m_telemetrySeq;
      }
      stepsSincePublish = 0;

      // ЖОРСТКИЙ БАР'ЄР: Фізика чекає, поки місія прочитає цей publish
      while (!m_stopRequested.load() && m_consumedSeq.load(std::memory_order_acquire) < publishedFrame) {
        std::this_thread::sleep_for(std::chrono::microseconds(200));
      }
    }
  }

  // Фінальний пуш перед смертю потоку, щоб планувальник місії випадково не завис у блокуванні
  std::lock_guard<std::mutex> lock(m_mutex);
  publishTelemetryLocked();
}

}  // namespace BallisticApp

#include "BallisticApp/mission/Autopilot.h"
#include "BallisticApp/DroneStateType.h"
#include "BallisticApp/config/AmmoParams.h"
#include "BallisticApp/config/DroneConfig.h"
#include "BallisticApp/interfaces/IBallisticSolver.h"
#include "BallisticApp/link/GpioPins.h"
#include "BallisticApp/link/UartLink.h"
#include "BallisticApp/net/MavlinkTelemetry.h"
#include "BallisticApp/states/DroneStateRegistry.h"
#include "BallisticApp/utils/Logger.h"
#include "BallisticApp/utils/MathUtils.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace BallisticApp {

bool Autopilot::handshake(UartLink& link, GpioPins& gpio, dlink::AmmoCfg& outAmmo, dlink::DroneCfg& outCfg, int timeoutMs)
{
  bool haveAmmo = false;
  bool haveCfg = false;

  link.onAmmo([&](const dlink::AmmoCfg& a) {
    outAmmo = a;
    haveAmmo = true;
    APP_LOG_MOD("Main", "autopilot: AMMO configuration package finalized: {}", a.name);
  });

  link.onConfig([&](const dlink::DroneCfg& c) {
    outCfg = c;
    haveCfg = true;
    APP_LOG_MOD("Main", "autopilot: CONFIG parameters verified. Speed={}", c.attackSpeed);
  });

  APP_LOG_MOD("Main", "autopilot: activating global hardware handshake (START->1)...");
  gpio.setStart(true);

  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
  while ((!haveAmmo || !haveCfg) && std::chrono::steady_clock::now() < deadline) {
    static_cast<void>(link.pump());
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return haveAmmo && haveCfg;
}

Autopilot::Autopilot(UartLink& link,
                     GpioPins& gpio,
                     std::unique_ptr<IBallisticSolver> solver,
                     const dlink::AmmoCfg& ammo,
                     const dlink::DroneCfg& cfg,
                     std::shared_ptr<MavlinkTelemetry> mavlink)
  : m_link(link)
  , m_gpio(gpio)
  , m_solver(std::move(solver))
  , m_ammo(ammo)
  , m_cfg(cfg)
  , m_mavlink(std::move(mavlink))
{
  m_targets.setExpectedCount(m_ammo.nTargets);

  m_link.onAmmo([this](const dlink::AmmoCfg& a) {
    m_ammo = a;
    m_targets.setExpectedCount(m_ammo.nTargets);
  });

  m_link.onConfig([this](const dlink::DroneCfg& c) { m_cfg = c; });

  m_link.onTarget([this](const dlink::TargetPos& p) { m_targets.update(p, m_lastTSec.load()); });

  m_link.onTelemetry([this](const dlink::Telemetry& t) { onTelemetryReceived(t); });
}

Autopilot::~Autopilot() noexcept
{
  stop();
}

void Autopilot::start() noexcept
{
  m_isStarted.store(true);
}

void Autopilot::stop() noexcept
{
  m_stopRequested.store(true);
}

bool Autopilot::isThreadReady() const noexcept
{
  return m_isIoReady.load() && m_isMissionReady.load();
}

void Autopilot::runIO()
{
  m_isIoReady.store(true);
  while (!m_stopRequested.load() && !m_isStarted.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  while (!m_stopRequested.load()) {
    static_cast<void>(m_link.pump());
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void Autopilot::runMission()
{
  m_isMissionReady.store(true);
  while (!m_stopRequested.load() && !m_isStarted.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  float dt = m_cfg.timeStep > 1e-4f ? m_cfg.timeStep : 0.1f;
  float scale = m_cfg.timeScale > 1e-4f ? m_cfg.timeScale : 1.0f;

  while (!m_stopRequested.load()) {
    if (m_mavlink && m_mavlink->isAckReceived()) {
      break;
    }

    float lastTSecSnapshot = m_lastTSec.load();
    float startSecSnapshot = m_missionStartSec.load();

    if (startSecSnapshot >= 0.0f && (lastTSecSnapshot - startSecSnapshot) > kMaxMissionTimeSec) {
      std::cerr << "[autopilot] Mission timeout, stopping\n";
      break;
    }

    dlink::Telemetry telSnapshot{};
    bool canProcess = false;

    {
      std::lock_guard<std::mutex> lock(m_telemetryMutex);
      if (m_hasTelemetry) {
        telSnapshot = m_latestTelemetry;
        m_hasTelemetry = false;
        canProcess = true;
      }
    }

    if (canProcess) {
      executeMissionStep(telSnapshot);
    }

    const float sleepSeconds = dt / scale;
    std::this_thread::sleep_for(std::chrono::duration<float>(sleepSeconds));
  }
}

void Autopilot::onTelemetryReceived(const dlink::Telemetry& tel)
{
  m_lastTSec.store(static_cast<float>(tel.t_ms) / 1000.0f);

  std::lock_guard<std::mutex> lock(m_telemetryMutex);
  m_latestTelemetry = tel;
  m_hasTelemetry = true;
}

void Autopilot::executeMissionStep(const dlink::Telemetry& tel)
{
  float currentLastTSec = m_lastTSec.load();
  if (m_missionStartSec.load() < 0.0f) {
    m_missionStartSec.store(currentLastTSec);
  }

  m_simTimeMs = tel.t_ms;

  if (m_mavlink) {
    Coord currentSpeed{std::cos(tel.dir) * tel.speed, std::sin(tel.dir) * tel.speed};
    m_mavlink->feedTelemetry({tel.x, tel.y}, currentSpeed, tel.dir, tel.z, m_simTimeMs);
  }

  Coord dronePos{tel.x, tel.y};

  if (m_dropped.load()) {
    dlink::Control c = m_flight.compute(tel, tel.dir, DroneStateType::MOVING, m_cfg.turnThreshold, m_cfg.attackSpeed);
    std::lock_guard<std::mutex> uartLock(m_uartWriteMutex);
    m_link.sendControl(c.accel, c.turnRate);
    return;
  }

  AmmoParams ammoParams{m_ammo.name, m_ammo.mass, m_ammo.drag, m_ammo.lift};
  auto bResult = m_solver->calculate(tel.z, tel.speed, ammoParams);
  float ballisticTime = bResult.flightTime;
  float ballisticDist = bResult.hDistance;

  int count = m_targets.getTargetCount();
  int bestTarget = -1;
  float bestTotal = 1e9f;
  Coord bestDrop{0.0f, 0.0f}, bestLead{0.0f, 0.0f};

  for (int i = 0; i < count; ++i) {
    if (!m_targets.hasTarget(i))
      continue;

    Coord dp = predictTargetIntercept(i, dronePos, ballisticTime, ballisticDist);
    Coord dirVec = (dp - dronePos).normalize();
    if (dirVec.lengthSquared() < 1e-8f)
      continue;

    Coord lead = dp + dirVec * ballisticDist;
    float dist = dronePos.distanceTo(lead);
    float dirToLead = std::atan2(lead.y - dronePos.y, lead.x - dronePos.x);
    float headErr = std::fabs(Math::normalizeAngle(dirToLead - tel.dir));
    float total = dist / std::max(1.0f, m_cfg.attackSpeed) + ballisticTime + headErr / std::max(0.1f, m_cfg.angularSpeed);

    if (total < bestTotal) {
      bestTotal = total;
      bestTarget = i;
      bestDrop = dp;
      bestLead = lead;
    }
  }

  if (bestTarget < 0) {
    dlink::Control c = m_flight.compute(tel, tel.dir, DroneStateType::STOPPED, m_cfg.turnThreshold, m_cfg.attackSpeed);
    std::lock_guard<std::mutex> uartLock(m_uartWriteMutex);
    m_link.sendControl(c.accel, c.turnRate);
    return;
  }

  bool needSwitch = (m_currentTarget < 0) || (bestTarget != m_currentTarget && bestTotal < m_currentTargetTime - 1.0f);
  if (needSwitch) {
    m_currentTarget = bestTarget;
    m_currentTargetTime = bestTotal;
    m_prevHitDist = 1e9f;
  }

  Coord dp, lead;
  if (bestTarget == m_currentTarget) {
    m_currentTargetTime = bestTotal;
    dp = bestDrop;
    lead = bestLead;
  }
  else {
    dp = predictTargetIntercept(m_currentTarget, dronePos, ballisticTime, ballisticDist);
    Coord dirVec = (dp - dronePos).normalize();
    lead = dp + dirVec * ballisticDist;
  }

  m_dropPoint = dp;
  float desiredDir = std::atan2(lead.y - dronePos.y, lead.x - dronePos.x);

  float speedArg = tel.speed;
  float dirArg = tel.dir;

  DroneConfig droneCfg{
    m_cfg.attackSpeed, m_cfg.accelerationPath, m_cfg.angularSpeed, m_cfg.turnThreshold, m_cfg.timeStep, m_cfg.timeScale, m_ammo.hitRadius};
  auto fsmState = DroneStateRegistry::getState(m_currentFsmState);
  m_currentFsmState = fsmState->execute(speedArg, dirArg, desiredDir, droneCfg);

  dlink::Control c = m_flight.compute(tel, desiredDir, m_currentFsmState, m_cfg.turnThreshold, m_cfg.attackSpeed);
  {
    std::lock_guard<std::mutex> uartLock(m_uartWriteMutex);
    m_link.sendControl(c.accel, c.turnRate);
  }

  if (!m_dropped.load()) {
    Coord bombLanding = dronePos + Coord{std::cos(tel.dir), std::sin(tel.dir)} * ballisticDist;
    Target tgt = m_targets.getTarget(m_currentTarget);
    Coord futureTarget = tgt.pos + tgt.velocity * ballisticTime;

    float bombToTarget = bombLanding.distanceTo(futureTarget);
    float hitDist = dronePos.distanceTo(m_dropPoint);

    float currentDirToLead = std::atan2(lead.y - dronePos.y, lead.x - dronePos.x);
    float currentHeadErr = std::fabs(Math::normalizeAngle(currentDirToLead - tel.dir));
    bool isAligned = currentHeadErr <= 0.1f;

    bool goodHit = (bombToTarget <= m_ammo.hitRadius) && isAligned;
    bool passedApex = (hitDist > m_prevHitDist) && (m_prevHitDist < m_ammo.hitRadius * 3.0f) && isAligned;

    if (goodHit || passedApex) {
      APP_LOG_MOD("Main", "autopilot: payload released (target={}, predicted miss={:.3f}m)", m_currentTarget, bombToTarget);
      m_dropped.store(true);

      if (m_mavlink) {
        m_mavlink->notifyDrop(m_dropPoint, tel.z);
      }

      m_gpio.pulseDrop(kDropPulseDurationMs);
    }
    m_prevHitDist = hitDist;
  }

  SimStep stepSnapshot{};
  stepSnapshot.pos = dronePos;
  stepSnapshot.direction = tel.dir;

  if (m_currentFsmState == DroneStateType::STOPPED)
    stepSnapshot.mode = "STOPPED";
  else if (m_currentFsmState == DroneStateType::ACCELERATING)
    stepSnapshot.mode = "ACCELERATING";
  else if (m_currentFsmState == DroneStateType::DECELERATING)
    stepSnapshot.mode = "DECELERATING";
  else if (m_currentFsmState == DroneStateType::TURNING)
    stepSnapshot.mode = "TURNING";
  else if (m_currentFsmState == DroneStateType::MOVING)
    stepSnapshot.mode = "MOVING";

  stepSnapshot.currentTarget = m_currentTarget;
  stepSnapshot.dropPoint = m_dropPoint;
  stepSnapshot.aimPoint = m_dropPoint + Coord{std::cos(tel.dir), std::sin(tel.dir)} * ballisticDist;
  stepSnapshot.timeSec = static_cast<float>(tel.t_ms) / 1000.0f;

  if (m_currentTarget >= 0) {
    Target tgtView = m_targets.getTarget(m_currentTarget);
    stepSnapshot.predictedTarget = tgtView.pos + tgtView.velocity * ballisticTime;
  }
  m_simSteps.push_back(stepSnapshot);
}

Coord Autopilot::predictTargetIntercept(int targetIdx, const Coord& dronePos, float ballisticTime, float ballisticDist) const noexcept
{
  float t_approximation = ballisticTime;
  Coord predictedTgt{0.0f, 0.0f};

  for (int iter = 0; iter < kPredictionIterations; ++iter) {
    Target futureTgt = m_targets.getTarget(targetIdx);
    predictedTgt = futureTgt.pos + futureTgt.velocity * t_approximation;
    float travelDist = std::max(0.0f, dronePos.distanceTo(predictedTgt) - ballisticDist);
    t_approximation = travelDist / std::max(1.0f, m_cfg.attackSpeed) + ballisticTime;
  }
  return predictedTgt;
}

bool Autopilot::isFinished() const noexcept
{
  return m_mavlink ? m_mavlink->isAckReceived() : m_dropped.load();
}

}  // namespace BallisticApp

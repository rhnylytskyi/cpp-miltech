#define _USE_MATH_DEFINES
#include "BallisticApp/mission/Autopilot.h"
#include "BallisticApp/states/DroneStateRegistry.h"
#include "BallisticApp/config/AmmoParams.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

namespace {
constexpr float kPi = 3.14159265358979323846f;

float normalizeAngleLocal(float a)
{
  while (a > kPi)
    a -= 2.0f * kPi;
  while (a < -kPi)
    a += 2.0f * kPi;
  return a;
}
}  // namespace

namespace BallisticApp::mission {

bool Autopilot::handshake(sys::UartLink& link, sys::GpioPins& gpio, dlink::AmmoCfg& outAmmo, dlink::DroneCfg& outCfg, int timeoutMs)
{
  bool haveAmmo = false, haveCfg = false;

  link.onAmmo([&](const dlink::AmmoCfg& a) {
    outAmmo = a;
    haveAmmo = true;
    std::cerr << "[autopilot] AMMO: " << a.name << " hitRadius=" << a.hitRadius << " nTargets=" << (int)a.nTargets << "\n";
  });

  link.onConfig([&](const dlink::DroneCfg& c) {
    outCfg = c;
    haveCfg = true;
    std::cerr << "[autopilot] CONFIG: attackSpeed=" << c.attackSpeed << " turnThreshold=" << c.turnThreshold
              << " angularSpeed=" << c.angularSpeed << "\n";
  });

  gpio.setStart(true);

  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
  while ((!haveAmmo || !haveCfg) && std::chrono::steady_clock::now() < deadline) {
    link.pump();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  if (!haveAmmo || !haveCfg) {
    std::cerr << "[autopilot] handshake timeout (ammo=" << haveAmmo << " cfg=" << haveCfg << ")\n";
  }

  return haveAmmo && haveCfg;
}

Autopilot::Autopilot(sys::UartLink& link,
                     sys::GpioPins& gpio,
                     std::unique_ptr<IBallisticSolver> solver,
                     const dlink::AmmoCfg& ammo,
                     const dlink::DroneCfg& cfg)
  : m_link(link)
  , m_gpio(gpio)
  , m_solver(std::move(solver))
  , m_ammo(ammo)
  , m_cfg(cfg)
{
  m_targets.setExpectedCount(m_ammo.nTargets);

  m_link.onTarget([this](const dlink::TargetPos& p) { m_targets.update(p, m_lastTSec); });

  m_link.onTelemetry([this](const dlink::Telemetry& t) {
    m_lastTSec = (float)t.t_ms / 1000.0f;
    onTelemetry(t);
  });
}

bool Autopilot::step()
{
  m_link.pump();
  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  if (m_missionStartSec >= 0.0f && (m_lastTSec - m_missionStartSec) > kMaxMissionTimeSec) {
    std::cerr << "[autopilot] mission timeout, stopping\n";
    return false;
  }

  return !m_dropped;
}

void Autopilot::onTelemetry(const dlink::Telemetry& tel)
{
  if (m_missionStartSec < 0.0f) {
    m_missionStartSec = m_lastTSec;
  }

  if (m_dropped) {
    dlink::Control c = m_flight.compute(tel, tel.dir, DroneStateType::MOVING, m_cfg.turnThreshold, m_cfg.attackSpeed);
    m_link.sendControl(c.accel, c.turnRate);
    return;
  }

  Coord dronePos{tel.x, tel.y};
  float altitude = tel.z;

  AmmoParams ammoParams;
  ammoParams.name = m_ammo.name;
  ammoParams.mass = m_ammo.mass;
  ammoParams.drag = m_ammo.drag;
  ammoParams.lift = m_ammo.lift;

  auto bResult = m_solver->calculate(altitude, tel.speed, ammoParams);
  float ballisticTime = bResult.flightTime;
  float ballisticDist = bResult.hDistance;

  int count = m_targets.getTargetCount();
  int bestTarget = -1;
  float bestTotal = 1e9f;
  Coord bestDrop{0.0f, 0.0f}, bestLead{0.0f, 0.0f};

  for (int i = 0; i < count; ++i) {
    if (!m_targets.hasTarget(i))
      continue;

    float t_approximation = ballisticTime;
    Coord predictedTgt;
    for (int iter = 0; iter < 8; ++iter) {
      sys::Target futureTgt = m_targets.getTarget(i);
      predictedTgt = futureTgt.pos + futureTgt.velocity * t_approximation;
      float travelDist = std::max(0.0f, (predictedTgt - dronePos).length() - ballisticDist);
      t_approximation = travelDist / m_cfg.attackSpeed + ballisticTime;
    }

    Coord dp = predictedTgt;
    Coord dirVec = (dp - dronePos).normalize();
    if (dirVec.length() < 1e-4f)
      continue;

    Coord lead = dp + dirVec * ballisticDist;
    float dist = dronePos.distanceTo(lead);
    float dirToLead = std::atan2(lead.y - dronePos.y, lead.x - dronePos.x);
    float headErr = std::fabs(normalizeAngleLocal(dirToLead - tel.dir));

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
    float t_approximation = ballisticTime;
    for (int iter = 0; iter < 8; ++iter) {
      sys::Target futureTgt = m_targets.getTarget(m_currentTarget);
      dp = futureTgt.pos + futureTgt.velocity * t_approximation;
      float travelDist = std::max(0.0f, (dp - dronePos).length() - ballisticDist);
      t_approximation = travelDist / m_cfg.attackSpeed + ballisticTime;
    }
    Coord dirVec = (dp - dronePos).normalize();
    lead = dp + dirVec * ballisticDist;
  }

  m_dropPoint = dp;
  float desiredDir = std::atan2(lead.y - dronePos.y, lead.x - dronePos.x);

  float speedArg = tel.speed;
  float dirArg = tel.dir;

  DroneConfig droneCfg;
  droneCfg.attackSpeed = m_cfg.attackSpeed;
  droneCfg.accelerationPath = m_cfg.accelerationPath;
  droneCfg.angularSpeed = m_cfg.angularSpeed;
  droneCfg.turnThreshold = m_cfg.turnThreshold;
  droneCfg.timeStep = m_cfg.timeStep;
  droneCfg.timeScale = m_cfg.timeScale;
  droneCfg.hitRadius = m_ammo.hitRadius;

  static DroneStateType currentFsmState = DroneStateType::STOPPED;
  auto fsmState = states::DroneStateRegistry::getState(currentFsmState);
  currentFsmState = fsmState->execute(speedArg, dirArg, desiredDir, droneCfg);

  dlink::Control c = m_flight.compute(tel, desiredDir, currentFsmState, m_cfg.turnThreshold, m_cfg.attackSpeed);
  m_link.sendControl(c.accel, c.turnRate);

  if (!m_dropped) {
    Coord bombLanding = dronePos + Coord{std::cos(tel.dir), std::sin(tel.dir)} * ballisticDist;
    sys::Target tgt = m_targets.getTarget(m_currentTarget);
    Coord futureTarget = tgt.pos + tgt.velocity * ballisticTime;

    float bombToTarget = bombLanding.distanceTo(futureTarget);
    float hitDist = dronePos.distanceTo(m_dropPoint);

    bool goodHit = bombToTarget <= m_ammo.hitRadius;
    bool passedApex = hitDist > m_prevHitDist && m_prevHitDist < m_ammo.hitRadius * 3.0f;

    if (goodHit || passedApex) {
      std::cerr << "[autopilot] DROP: target=" << m_currentTarget << " miss~" << bombToTarget << "m t=" << m_lastTSec << "s\n";
      m_gpio.pulseDrop(80);
      m_dropped = true;
    }
    m_prevHitDist = hitDist;
  }
}

}  // namespace BallisticApp::mission

#include "BallisticApp/mission/Autopilot.h"
#include "BallisticApp/utils/MathUtils.h"
#include "BallisticApp/states/DroneStateRegistry.h"
#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/config/AmmoParams.h"
#include "BallisticApp/config/DroneConfig.h"
#include "BallisticApp/interfaces/IBallisticSolver.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

namespace BallisticApp::mission {

bool Autopilot::handshake(sys::UartLink& link, sys::GpioPins& gpio, dlink::AmmoCfg& outAmmo, dlink::DroneCfg& outCfg, int timeoutMs)
{
  bool haveAmmo = false;
  bool haveCfg = false;

  link.onAmmo([&](const dlink::AmmoCfg& a) {
    outAmmo = a;
    haveAmmo = true;
    std::cerr << "[autopilot] AMMO received: " << a.name << "\n";
  });

  link.onConfig([&](const dlink::DroneCfg& c) {
    outCfg = c;
    haveCfg = true;
    std::cerr << "[autopilot] CONFIG received: attackSpeed=" << c.attackSpeed << "\n";
  });

  std::cerr << "[autopilot] Set START line high...\n";
  gpio.setStart(true);

  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
  while ((!haveAmmo || !haveCfg) && std::chrono::steady_clock::now() < deadline) {
    static_cast<void>(link.pump());
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
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

  // Maintain persistent subscription callbacks so configurations aren't dropped post-handshake
  m_link.onAmmo([this](const dlink::AmmoCfg& a) {
    m_ammo = a;
    m_targets.setExpectedCount(m_ammo.nTargets);
  });

  m_link.onConfig([this](const dlink::DroneCfg& c) { m_cfg = c; });

  m_link.onTarget([this](const dlink::TargetPos& p) { m_targets.update(p, m_lastTSec); });

  m_link.onTelemetry([this](const dlink::Telemetry& t) {
    m_lastTSec = static_cast<float>(t.t_ms) / 1000.0f;
    onTelemetry(t);
  });
}

Coord Autopilot::predictTargetIntercept(int targetIdx, const Coord& dronePos, float ballisticTime, float ballisticDist) const noexcept
{
  float t_approximation = ballisticTime;
  Coord predictedTgt{0.0f, 0.0f};

  for (int iter = 0; iter < kPredictionIterations; ++iter) {
    sys::Target futureTgt = m_targets.getTarget(targetIdx);
    predictedTgt = futureTgt.pos + futureTgt.velocity * t_approximation;
    float travelDist = std::max(0.0f, dronePos.distanceTo(predictedTgt) - ballisticDist);
    t_approximation = travelDist / std::max(1.0f, m_cfg.attackSpeed) + ballisticTime;
  }
  return predictedTgt;
}

bool Autopilot::step()
{
  static_cast<void>(m_link.pump());
  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  if (m_missionStartSec >= 0.0f && (m_lastTSec - m_missionStartSec) > kMaxMissionTimeSec) {
    std::cerr << "[autopilot] Mission timeout, stopping\n";
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

  auto fsmState = states::DroneStateRegistry::getState(m_currentFsmState);
  m_currentFsmState = fsmState->execute(speedArg, dirArg, desiredDir, droneCfg);

  dlink::Control c = m_flight.compute(tel, desiredDir, m_currentFsmState, m_cfg.turnThreshold, m_cfg.attackSpeed);
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
      std::cerr << "[autopilot] DROP: target=" << m_currentTarget << " miss~" << bombToTarget << "m\n";
      m_gpio.pulseDrop(kDropPulseDurationMs);
      m_dropped = true;
    }
    m_prevHitDist = hitDist;
  }
}

}  // namespace BallisticApp::mission

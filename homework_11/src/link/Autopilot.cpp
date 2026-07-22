#define _USE_MATH_DEFINES
#include "BallisticApp/link/Autopilot.h"
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
  while (a > kPi) {
    a -= 2.0f * kPi;
  }
  while (a < -kPi) {
    a += 2.0f * kPi;
  }
  return a;
}
}  // namespace

namespace BallisticApp {

bool Autopilot::handshake(UartLink& link, GpioPins& gpio, dlink::AmmoCfg& outAmmo, dlink::DroneCfg& outCfg, int timeoutMs)
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

Autopilot::Autopilot(
  UartLink& link, GpioPins& gpio, std::unique_ptr<IBallisticSolver> solver, const dlink::AmmoCfg& ammo, const dlink::DroneCfg& cfg)
  : link_(link)
  , gpio_(gpio)
  , solver_(std::move(solver))
  , ammo_(ammo)
  , cfg_(cfg)
{
  targets_.setExpectedCount(ammo_.nTargets);

  link_.onTarget([this](const dlink::TargetPos& p) { targets_.update(p, lastTSec_); });

  link_.onTelemetry([this](const dlink::Telemetry& t) {
    lastTSec_ = (float)t.t_ms / 1000.0f;
    onTelemetry(t);
  });
}

bool Autopilot::step()
{
  link_.pump();
  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  if (missionStartSec_ >= 0.0f && (lastTSec_ - missionStartSec_) > kMaxMissionTimeSec) {
    std::cerr << "[autopilot] mission timeout, stopping\n";
    return false;
  }

  return !dropped_;
}

void Autopilot::onTelemetry(const dlink::Telemetry& tel)
{
  if (missionStartSec_ < 0.0f) {
    missionStartSec_ = lastTSec_;
  }

  if (dropped_) {
    dlink::Control c = flight_.compute(tel, tel.dir, DroneStateType::MOVING, cfg_.turnThreshold, cfg_.attackSpeed);
    link_.sendControl(c.accel, c.turnRate);
    return;
  }

  Coord dronePos{tel.x, tel.y};
  float altitude = tel.z;

  // Наповнюємо вашу оригінальну структуру для сумісності з вашим calculate()
  AmmoParams ammoParams;
  ammoParams.name = ammo_.name;
  ammoParams.mass = ammo_.mass;
  ammoParams.drag = ammo_.drag;
  ammoParams.lift = ammo_.lift;

  // Викликаємо ваш рідний оригінальний метод солвера
  auto bResult = solver_->calculate(altitude, tel.speed, ammoParams);
  float ballisticTime = bResult.flightTime;
  float ballisticDist = bResult.hDistance;

  int count = targets_.getTargetCount();
  int bestTarget = -1;
  float bestTotal = 1e9f;
  Coord bestDrop{0.0f, 0.0f}, bestLead{0.0f, 0.0f};

  // 8-ітераційний цикл розрахунку упередження (залишається у тілі автопілота)
  for (int i = 0; i < count; ++i) {
    if (!targets_.hasTarget(i)) {
      continue;
    }

    float t_approximation = ballisticTime;
    Coord predictedTgt;
    for (int iter = 0; iter < 8; ++iter) {
      Target futureTgt = targets_.getTarget(i);
      predictedTgt = futureTgt.pos + futureTgt.velocity * t_approximation;
      float travelDist = std::max(0.0f, (predictedTgt - dronePos).length() - ballisticDist);
      t_approximation = travelDist / cfg_.attackSpeed + ballisticTime;
    }

    Coord dp = predictedTgt;
    Coord dirVec = (dp - dronePos).normalize();
    if (dirVec.length() < 1e-4f) {
      continue;
    }

    Coord lead = dp + dirVec * ballisticDist;
    float dist = dronePos.distanceTo(lead);
    float dirToLead = std::atan2(lead.y - dronePos.y, lead.x - dronePos.x);
    float headErr = std::fabs(normalizeAngleLocal(dirToLead - tel.dir));

    float total = dist / std::max(1.0f, cfg_.attackSpeed) + ballisticTime + headErr / std::max(0.1f, cfg_.angularSpeed);

    if (total < bestTotal) {
      bestTotal = total;
      bestTarget = i;
      bestDrop = dp;
      bestLead = lead;
    }
  }

  if (bestTarget < 0) {
    dlink::Control c = flight_.compute(tel, tel.dir, DroneStateType::STOPPED, cfg_.turnThreshold, cfg_.attackSpeed);
    link_.sendControl(c.accel, c.turnRate);
    return;
  }

  // Логіка гістерезису утримання цілі
  bool needSwitch = (currentTarget_ < 0) || (bestTarget != currentTarget_ && bestTotal < currentTargetTime_ - 1.0f);
  if (needSwitch) {
    currentTarget_ = bestTarget;
    currentTargetTime_ = bestTotal;
    prevHitDist_ = 1e9f;
  }

  Coord dp, lead;
  if (bestTarget == currentTarget_) {
    currentTargetTime_ = bestTotal;
    dp = bestDrop;
    lead = bestLead;
  }
  else {
    float t_approximation = ballisticTime;
    for (int iter = 0; iter < 8; ++iter) {
      Target futureTgt = targets_.getTarget(currentTarget_);
      dp = futureTgt.pos + futureTgt.velocity * t_approximation;
      float travelDist = std::max(0.0f, (dp - dronePos).length() - ballisticDist);
      t_approximation = travelDist / cfg_.attackSpeed + ballisticTime;
    }
    Coord dirVec = (dp - dronePos).normalize();
    lead = dp + dirVec * ballisticDist;
  }

  dropPoint_ = dp;
  float desiredDir = std::atan2(lead.y - dronePos.y, lead.x - dronePos.x);

  // Логіка Кінцевого Автомата (FSM)
  float speedArg = tel.speed;
  float dirArg = tel.dir;

  DroneConfig droneCfg;
  droneCfg.attackSpeed = cfg_.attackSpeed;
  droneCfg.accelerationPath = cfg_.accelerationPath;
  droneCfg.angularSpeed = cfg_.angularSpeed;
  droneCfg.turnThreshold = cfg_.turnThreshold;
  droneCfg.timeStep = cfg_.timeStep;
  droneCfg.timeScale = cfg_.timeScale;
  droneCfg.hitRadius = ammo_.hitRadius;

  static DroneStateType currentFsmState = DroneStateType::STOPPED;
  auto fsmState = DroneStateRegistry::getState(currentFsmState);
  currentFsmState = fsmState->execute(speedArg, dirArg, desiredDir, droneCfg);

  // Виклик польотного контролера
  dlink::Control c = flight_.compute(tel, desiredDir, currentFsmState, cfg_.turnThreshold, cfg_.attackSpeed);
  link_.sendControl(c.accel, c.turnRate);

  // Перевірка умови скидання
  if (!dropped_) {
    Coord bombLanding = dronePos + Coord{std::cos(tel.dir), std::sin(tel.dir)} * ballisticDist;
    Target tgt = targets_.getTarget(currentTarget_);
    Coord futureTarget = tgt.pos + tgt.velocity * ballisticTime;

    float bombToTarget = bombLanding.distanceTo(futureTarget);
    float hitDist = dronePos.distanceTo(dropPoint_);

    bool goodHit = bombToTarget <= ammo_.hitRadius;
    bool passedApex = hitDist > prevHitDist_ && prevHitDist_ < ammo_.hitRadius * 3.0f;

    if (goodHit || passedApex) {
      std::cerr << "[autopilot] DROP: target=" << currentTarget_ << " miss~" << bombToTarget << "m t=" << lastTSec_ << "s\n";
      gpio_.pulseDrop(80);
      dropped_ = true;
    }
    prevHitDist_ = hitDist;
  }
}

}  // namespace BallisticApp
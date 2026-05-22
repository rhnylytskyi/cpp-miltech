#include "ballistic_app/MissionProcessor.hpp"
#include "ballistic_app/Defines.hpp"
#include "ballistic_app/utils/MathUtils.hpp"
#include "ballistic_app//io/interfaces/IConfigLoader.hpp"
#include "ballistic_app//io/interfaces/ISimulationExporter.hpp"
#include <iostream>
#include <cmath>

namespace BallisticApp {

MissionProcessor::MissionProcessor(IConfigLoader* loader,
                                   ITargetProvider* provider,
                                   IBallisticSolver* solver,
                                   ISimulationExporter* exporter)
  : m_loader(loader)
  , m_provider(provider)
  , m_solver(solver)
  , m_exporter(exporter)
  , steps(nullptr)
  , dronePos{0, 0}
  , direction(0.0f)
  , speed(0.0f)
  , state(DroneState::STOPPED)
  , currentTime(0.0f)
  , totalSteps(0)
  , isMissionFinished(false)
{
  if (!loader || !provider || !solver || !exporter) {
    throw std::runtime_error("MissionProcessor Error: Failed to create critical application components.");
  }
  reset();
}

MissionProcessor::~MissionProcessor()
{
  if (steps) {
    delete[] steps;
    steps = nullptr;
  }
  if (m_loader) {
    delete m_loader;
    m_loader = nullptr;
  }
  if (m_provider) {
    delete m_provider;
    m_provider = nullptr;
  }
  if (m_solver) {
    delete m_solver;
    m_solver = nullptr;
  }
  if (m_exporter) {
    delete m_exporter;
    m_exporter = nullptr;
  }
}

void MissionProcessor::init(const char* configSource, const char* ammoSource)
{
  if (m_loader) {
    m_loader->load(configSource, ammoSource);
    config = m_loader->getConfig();
    ammo = m_loader->getAmmoParams();
  }
  if (steps) {
    delete[] steps;
  }
  steps = new SimStep[MAX_STEPS];
  reset();

  // КЕШУВАННЯ БАЛІСТИКИ: Розраховуємо один раз при старті
  if (m_solver) {
    cachedFlightTime = m_solver->calcTimeOfFall(config.altitude, config.attackSpeed, ammo);
    cachedHDist = m_solver->calcHDistance(cachedFlightTime, config.attackSpeed, ammo);
  }

  LOG("Mission initialized. Ammo: " << ammo.name);
}

bool MissionProcessor::hasNext()
{
  return (totalSteps < MAX_STEPS) && !isMissionFinished;
}

// ============================================================
// ОНОВЛЕННЯ ФІЗИКИ: Єдине місце розрахунку логіки руху дрона
// ============================================================
void MissionProcessor::updateDronePhysics(DronePhysicsState& drone, const Coord& firePoint, float dt, float& outDeltaPath)
{
  float desiredDirection = std::atan2(firePoint.y - drone.pos.y, firePoint.x - drone.pos.x);
  float deltaAngle = Math::normalizeAngle(desiredDirection - drone.direction);
  float acceleration = (config.attackSpeed * config.attackSpeed) / (2.0f * config.accelPath);
  outDeltaPath = 0.0f;

  switch (drone.state) {
    case DroneState::STOPPED:
      if (std::fabs(deltaAngle) > config.turnThreshold)
        drone.state = DroneState::TURNING;
      else {
        drone.direction = desiredDirection;
        drone.state = DroneState::ACCELERATING;
      }
      break;
    case DroneState::ACCELERATING:
      if (std::fabs(deltaAngle) > config.turnThreshold && drone.speed > 0.01f) {
        drone.state = DroneState::DECELERATING;
        float prevSpeed = drone.speed;
        drone.speed -= acceleration * dt;
        if (drone.speed <= 0) {
          drone.speed = 0;
          drone.state = DroneState::STOPPED;
        }
        outDeltaPath = (prevSpeed + drone.speed) / 2.0f * dt;
      }
      else {
        if (std::fabs(deltaAngle) <= config.turnThreshold)
          drone.direction = desiredDirection;
        float prevSpeed = drone.speed;
        drone.speed += acceleration * dt;
        if (drone.speed >= config.attackSpeed) {
          drone.speed = config.attackSpeed;
          drone.state = DroneState::MOVING;
        }
        outDeltaPath = (prevSpeed + drone.speed) / 2.0f * dt;
      }
      break;
    case DroneState::DECELERATING: {
      float prevSpeed = drone.speed;
      drone.speed -= acceleration * dt;
      if (drone.speed <= 0) {
        drone.speed = 0;
        drone.state = DroneState::STOPPED;
      }
      outDeltaPath = (prevSpeed + drone.speed) / 2.0f * dt;
    } break;
    case DroneState::TURNING: {
      float da = Math::normalizeAngle(desiredDirection - drone.direction);
      if (std::fabs(da) <= config.angularSpeed * dt) {
        drone.direction = desiredDirection;
        drone.state = DroneState::ACCELERATING;
      }
      else {
        drone.direction += (da > 0 ? 1.0f : -1.0f) * config.angularSpeed * dt;
        drone.direction = Math::normalizeAngle(drone.direction);
      }
    } break;
    case DroneState::MOVING:
      if (std::fabs(deltaAngle) > config.turnThreshold) {
        drone.state = DroneState::DECELERATING;
        float prevSpeed = drone.speed;
        drone.speed -= acceleration * dt;
        if (drone.speed <= 0) {
          drone.speed = 0;
          drone.state = DroneState::STOPPED;
        }
        outDeltaPath = (prevSpeed + drone.speed) / 2.0f * dt;
      }
      else {
        if (std::fabs(deltaAngle) <= config.turnThreshold)
          drone.direction = desiredDirection;
        outDeltaPath = drone.speed * dt;
      }
      break;
  }

  drone.pos.x += std::cos(drone.direction) * outDeltaPath;
  drone.pos.y += std::sin(drone.direction) * outDeltaPath;
}

// ============================================================
// ПРОГНОЗУВАННЯ: Швидка симуляція вперед у пам'яті
// ============================================================
float MissionProcessor::predictTimeAndPos(int targetIdx, Coord& outFirePoint, Coord& outPredictedTarget)
{
  DronePhysicsState vDrone{dronePos, speed, direction, state};

  float vTime = currentTime;
  float elapsedPredictionTime = 0.0f;
  const float MAX_PREDICT_TIME = 30.0f;
  float dt = config.simTimeStep;

  while (elapsedPredictionTime < MAX_PREDICT_TIME) {
    outPredictedTarget = extrapolateTarget(targetIdx, vTime, cachedFlightTime);

    Coord delta = outPredictedTarget - vDrone.pos;
    outFirePoint = outPredictedTarget - Math::normalize(delta) * cachedHDist;

    if (vDrone.state == DroneState::MOVING && Math::length(vDrone.pos - outFirePoint) <= config.hitRadius * 0.25f) {
      return elapsedPredictionTime;
    }

    float deltaPath = 0.0f;
    updateDronePhysics(vDrone, outFirePoint, dt, deltaPath);

    vTime += dt;
    elapsedPredictionTime += dt;
  }

  return MAX_PREDICT_TIME;
}

// ============================================================
// ГОЛОВНИЙ КРОК СИМУЛЯЦІЇ
// ============================================================
SimStep MissionProcessor::step()
{
  float bestTime = 1e9f;
  int bestTarget = 0;
  Coord bestPredicted{0, 0};
  Coord bestFirePoint{0, 0};

  // Пошук оптимальної цілі з урахуванням маневреності дрона
  for (int tId = 0; tId < m_provider->getTargetCount(); ++tId) {
    Coord currentFirePoint{0, 0};
    Coord currentPredictedTarget{0, 0};

    float predictedTime = predictTimeAndPos(tId, currentFirePoint, currentPredictedTarget);

    if (predictedTime < bestTime) {
      bestTime = predictedTime;
      bestTarget = tId;
      bestPredicted = currentPredictedTarget;
      bestFirePoint = currentFirePoint;
    }
  }

  Coord firePoint = bestFirePoint;
  Coord aimPoint = dronePos + Coord{std::cos(direction), std::sin(direction)} * cachedHDist;

  // Логування історії поточного кроку
  steps[totalSteps].pos = dronePos;
  steps[totalSteps].direction = direction;
  steps[totalSteps].state = (int)state;
  steps[totalSteps].targetIdx = bestTarget;
  steps[totalSteps].dropPoint = firePoint;
  steps[totalSteps].aimPoint = aimPoint;
  steps[totalSteps].predictedTarget = bestPredicted;

  // Реальне виконання кроку фізики для дрона
  DronePhysicsState realDrone{dronePos, speed, direction, state};
  float deltaPath = 0.0f;

  updateDronePhysics(realDrone, firePoint, config.simTimeStep, deltaPath);

  dronePos = realDrone.pos;
  speed = realDrone.speed;
  direction = realDrone.direction;
  state = realDrone.state;

  // Перевірка критерію скидання снаряда
  if (state == DroneState::MOVING && Math::length(dronePos - firePoint) <= config.hitRadius * 0.25f) {
    isMissionFinished = true;
    LOG("Target captured. Weapon released at step: " << totalSteps);
  }

  currentTime += config.simTimeStep;
  SimStep currentStepData = steps[totalSteps];
  totalSteps++;
  return currentStepData;
}

void MissionProcessor::run()
{
  while (this->hasNext()) {
    this->step();
  }

  m_exporter->exportSimulation(getStepsHistory(), getTotalSteps());
}

void MissionProcessor::reset()
{
  dronePos = config.startPos;
  direction = config.initialDir;
  speed = 0.0f;
  state = DroneState::STOPPED;
  currentTime = 0.0f;
  totalSteps = 0;
  isMissionFinished = false;
}

void MissionProcessor::changeSolver(IBallisticSolver* solver)
{
  if (solver) {
    m_solver = solver;
    cachedFlightTime = m_solver->calcTimeOfFall(config.altitude, config.attackSpeed, ammo);
    cachedHDist = m_solver->calcHDistance(cachedFlightTime, config.attackSpeed, ammo);
  }
}

int MissionProcessor::getTotalSteps() const
{
  return totalSteps;
}

const SimStep* MissionProcessor::getStepsHistory() const
{
  return steps;
}

Coord MissionProcessor::interpolateTarget(int targetIdx, float t)
{
  int stepsCount = m_provider->getTimeSteps();
  if (stepsCount == 0)
    return {0, 0};

  int idx = (int)std::floor(t / config.arrayTimeStep) % stepsCount;
  int next = (idx + 1) % stepsCount;
  float frac = (t / config.arrayTimeStep) - std::floor(t / config.arrayTimeStep);

  Coord pIdx = m_provider->getTargetPos(targetIdx, idx);
  Coord pNext = m_provider->getTargetPos(targetIdx, next);

  return pIdx + (pNext - pIdx) * frac;
}

Coord MissionProcessor::extrapolateTarget(int targetIdx, float time, float dt)
{
  int stepsCount = m_provider->getTimeSteps();
  if (stepsCount == 0)
    return {0, 0};

  int idx = (int)std::floor(time / config.arrayTimeStep) % stepsCount;
  int next = (idx + 1) % stepsCount;

  Coord pIdx = m_provider->getTargetPos(targetIdx, idx);
  Coord pNext = m_provider->getTargetPos(targetIdx, next);

  Coord v = (pNext - pIdx) / config.arrayTimeStep;
  Coord curPos = interpolateTarget(targetIdx, time);

  return curPos + v * dt;
}

}  // namespace BallisticApp

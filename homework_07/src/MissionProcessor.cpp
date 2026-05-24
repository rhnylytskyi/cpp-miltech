#include "ballistic_app/MissionProcessor.h"
#include "ballistic_app/Defines.h"
#include "ballistic_app/utils/MathUtils.h"
#include "ballistic_app/interfaces/IConfigLoader.h"
#include "ballistic_app/interfaces/ITargetProvider.h"
#include "ballistic_app/interfaces/IBallisticSolver.h"
#include "ballistic_app/interfaces/ISimulationExporter.h"

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
  , m_steps(nullptr)
  , m_dronePos{0, 0}
  , m_direction(0.0f)
  , m_speed(0.0f)
  , m_state(DroneState::STOPPED)
  , m_currentTime(0.0f)
  , m_totalSteps(0)
  , m_isMissionFinished(false)
{
  if (!m_loader || !m_provider || !m_solver || !m_exporter) {
    throw std::runtime_error("MissionProcessor Error: Failed to create critical application components.");
  }
  reset();
}

MissionProcessor::~MissionProcessor()
{
  if (m_steps) {
    delete[] m_steps;
    m_steps = nullptr;
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
    m_config = m_loader->getConfig();
    m_ammo = m_loader->getAmmoParams();
  }
  if (m_steps) {
    delete[] m_steps;
  }
  m_steps = new SimStep[MAX_STEPS];
  reset();

  // КЕШУВАННЯ БАЛІСТИКИ: Розраховуємо один раз при старті
  if (m_solver) {
    m_cachedFlightTime = m_solver->calcTimeOfFall(m_config.altitude, m_config.attackSpeed, m_ammo);
    m_cachedHDist = m_solver->calcHDistance(m_cachedFlightTime, m_config.attackSpeed, m_ammo);
  }

  LOG("Mission initialized. Ammo: " << m_ammo.name);
}

bool MissionProcessor::hasNext()
{
  return (m_totalSteps < MAX_STEPS) && !m_isMissionFinished;
}

// ============================================================
// ОНОВЛЕННЯ ФІЗИКИ: Єдине місце розрахунку логіки руху дрона
// ============================================================
void MissionProcessor::updateDronePhysics(DronePhysicsState& drone, const Coord& firePoint, float dt, float& outDeltaPath)
{
  float desiredDirection = std::atan2(firePoint.y - drone.pos.y, firePoint.x - drone.pos.x);
  float deltaAngle = Math::normalizeAngle(desiredDirection - drone.direction);
  float acceleration = (m_config.attackSpeed * m_config.attackSpeed) / (2.0f * m_config.accelPath);
  outDeltaPath = 0.0f;

  switch (drone.state) {
    case DroneState::STOPPED:
      if (std::fabs(deltaAngle) > m_config.turnThreshold)
        drone.state = DroneState::TURNING;
      else {
        drone.direction = desiredDirection;
        drone.state = DroneState::ACCELERATING;
      }
      break;
    case DroneState::ACCELERATING:
      if (std::fabs(deltaAngle) > m_config.turnThreshold && drone.speed > 0.01f) {
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
        if (std::fabs(deltaAngle) <= m_config.turnThreshold)
          drone.direction = desiredDirection;
        float prevSpeed = drone.speed;
        drone.speed += acceleration * dt;
        if (drone.speed >= m_config.attackSpeed) {
          drone.speed = m_config.attackSpeed;
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
      if (std::fabs(da) <= m_config.angularSpeed * dt) {
        drone.direction = desiredDirection;
        drone.state = DroneState::ACCELERATING;
      }
      else {
        drone.direction += (da > 0 ? 1.0f : -1.0f) * m_config.angularSpeed * dt;
        drone.direction = Math::normalizeAngle(drone.direction);
      }
    } break;
    case DroneState::MOVING:
      if (std::fabs(deltaAngle) > m_config.turnThreshold) {
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
        if (std::fabs(deltaAngle) <= m_config.turnThreshold)
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
  DronePhysicsState vDrone{m_dronePos, m_speed, m_direction, m_state};

  float vTime = m_currentTime;
  float elapsedPredictionTime = 0.0f;
  const float MAX_PREDICT_TIME = 30.0f;
  float dt = m_config.simTimeStep;

  while (elapsedPredictionTime < MAX_PREDICT_TIME) {
    outPredictedTarget = extrapolateTarget(targetIdx, vTime, m_cachedFlightTime);

    Coord delta = outPredictedTarget - vDrone.pos;
    outFirePoint = outPredictedTarget - Math::normalize(delta) * m_cachedHDist;

    if (vDrone.state == DroneState::MOVING && Math::length(vDrone.pos - outFirePoint) <= m_config.hitRadius * 0.25f) {
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
  Coord aimPoint = m_dronePos + Coord{std::cos(m_direction), std::sin(m_direction)} * m_cachedHDist;

  // Логування історії поточного кроку
  m_steps[m_totalSteps].pos = m_dronePos;
  m_steps[m_totalSteps].direction = m_direction;
  m_steps[m_totalSteps].state = m_state;
  m_steps[m_totalSteps].targetIdx = bestTarget;
  m_steps[m_totalSteps].dropPoint = firePoint;
  m_steps[m_totalSteps].aimPoint = aimPoint;
  m_steps[m_totalSteps].predictedTarget = bestPredicted;

  // Реальне виконання кроку фізики для дрона
  DronePhysicsState realDrone{m_dronePos, m_speed, m_direction, m_state};
  float deltaPath = 0.0f;

  updateDronePhysics(realDrone, firePoint, m_config.simTimeStep, deltaPath);

  m_dronePos = realDrone.pos;
  m_speed = realDrone.speed;
  m_direction = realDrone.direction;
  m_state = realDrone.state;

  // Перевірка критерію скидання снаряда
  if (m_state == DroneState::MOVING && Math::length(m_dronePos - firePoint) <= m_config.hitRadius * 0.25f) {
    m_isMissionFinished = true;
    LOG("Target captured. Weapon released at step: " << m_totalSteps);
  }

  m_currentTime += m_config.simTimeStep;
  SimStep currentStepData = m_steps[m_totalSteps];
  m_totalSteps++;
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
  m_dronePos = m_config.startPos;
  m_direction = m_config.initialDir;
  m_speed = 0.0f;
  m_state = DroneState::STOPPED;
  m_currentTime = 0.0f;
  m_totalSteps = 0;
  m_isMissionFinished = false;
}

void MissionProcessor::changeSolver(IBallisticSolver* solver)
{
  if (solver) {
    m_solver = solver;
    m_cachedFlightTime = m_solver->calcTimeOfFall(m_config.altitude, m_config.attackSpeed, m_ammo);
    m_cachedHDist = m_solver->calcHDistance(m_cachedFlightTime, m_config.attackSpeed, m_ammo);
  }
}

int MissionProcessor::getTotalSteps() const
{
  return m_totalSteps;
}

const SimStep* MissionProcessor::getStepsHistory() const
{
  return m_steps;
}

Coord MissionProcessor::interpolateTarget(int targetIdx, float t)
{
  int stepsCount = m_provider->getTimeSteps();
  if (stepsCount == 0)
    return {0, 0};

  int idx = (int)std::floor(t / m_config.arrayTimeStep) % stepsCount;
  int next = (idx + 1) % stepsCount;
  float frac = (t / m_config.arrayTimeStep) - std::floor(t / m_config.arrayTimeStep);

  Coord pIdx = m_provider->getTargetPos(targetIdx, idx);
  Coord pNext = m_provider->getTargetPos(targetIdx, next);

  return pIdx + (pNext - pIdx) * frac;
}

Coord MissionProcessor::extrapolateTarget(int targetIdx, float time, float dt)
{
  int stepsCount = m_provider->getTimeSteps();
  if (stepsCount == 0)
    return {0, 0};

  int idx = (int)std::floor(time / m_config.arrayTimeStep) % stepsCount;
  int next = (idx + 1) % stepsCount;

  Coord pIdx = m_provider->getTargetPos(targetIdx, idx);
  Coord pNext = m_provider->getTargetPos(targetIdx, next);

  Coord v = (pNext - pIdx) / m_config.arrayTimeStep;
  Coord curPos = interpolateTarget(targetIdx, time);

  return curPos + v * dt;
}

}  // namespace BallisticApp

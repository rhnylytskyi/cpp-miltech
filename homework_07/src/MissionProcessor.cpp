#include "ballistic_app/MissionProcessor.hpp"
#include "ballistic_app/interfaces/IConfigLoader.hpp"
#include "ballistic_app/interfaces/ITargetProvider.hpp"
#include "ballistic_app/interfaces/IBallisticSolver.hpp"
#include "ballistic_app/Defines.hpp"
#include "ballistic_app/utils/MathUtils.hpp"
#include <iostream> // IWYU pragma: keep
#include <cmath>

namespace BallisticApp {

MissionProcessor::MissionProcessor(IConfigLoader* l, ITargetProvider* p, IBallisticSolver* s)
  : loader(l)
  , provider(p)
  , solver(s)
  , steps(nullptr)
{
  reset();
}

MissionProcessor::~MissionProcessor()
{
  if (steps) {
    delete[] steps;
    steps = nullptr;
  }
}

void MissionProcessor::init(const char* configSource, const char* ammoSource)
{
  if (loader) {
    loader->load(configSource, ammoSource);
    config = loader->getConfig();
    ammo = loader->getAmmoParams();
  }
  if (steps) {
    delete[] steps;
  }
  steps = new SimStep[MAX_STEPS];
  reset();
  LOG("Mission initialized. Ammo: " << ammo.name);
}

bool MissionProcessor::hasNext() 
{ 
  return (totalSteps < MAX_STEPS) && !isMissionFinished; 
}

SimStep MissionProcessor::step()
{
  float bestTime = 1e9f;
  int bestTarget = 0;
  Coord bestPredicted{0, 0};

  float flightTime = solver->calcTimeOfFall(config.altitude, config.attackSpeed, ammo);
  float hDist = solver->calcHDistance(flightTime, config.attackSpeed, ammo);

  for (int tId = 0; tId < provider->getTargetCount(); ++tId) {
    Coord predPos = {0, 0};
    float totalTime = 0.0f;
    for (int iter = 0; iter < 10; iter++) {
      predPos = extrapolateTarget(tId, currentTime, totalTime + flightTime);
      totalTime = Math::length(predPos - dronePos) / (config.attackSpeed + 0.1f);
    }
    if (totalTime < bestTime) {
      bestTime = totalTime;
      bestTarget = tId;
      bestPredicted = predPos;
    }
  }

  Coord delta = bestPredicted - dronePos;
  Coord firePoint = bestPredicted - Math::normalize(delta) * hDist;
  Coord aimPoint = dronePos + Coord{std::cos(direction), std::sin(direction)} * hDist;

  steps[totalSteps].pos = dronePos;
  steps[totalSteps].direction = direction;
  steps[totalSteps].state = (int)state;
  steps[totalSteps].targetIdx = bestTarget;
  steps[totalSteps].dropPoint = firePoint;
  steps[totalSteps].aimPoint = aimPoint;
  steps[totalSteps].predictedTarget = bestPredicted;

  float desiredDirection = std::atan2(firePoint.y - dronePos.y, firePoint.x - dronePos.x);
  float deltaAngle = Math::normalizeAngle(desiredDirection - direction);
  float acceleration = (config.attackSpeed * config.attackSpeed) / (2.0f * config.accelPath);
  float deltaPath = 0.0f;

  switch (state) {
    case DroneState::STOPPED:
      if (std::fabs(deltaAngle) > config.turnThreshold)
        state = DroneState::TURNING;
      else {
        direction = desiredDirection;
        state = DroneState::ACCELERATING;
      }
      break;
    case DroneState::ACCELERATING:
      if (std::fabs(deltaAngle) > config.turnThreshold && speed > 0.01f) {
        state = DroneState::DECELERATING;
        float prevSpeed = speed;
        speed -= acceleration * config.simTimeStep;
        if (speed <= 0) {
          speed = 0;
          state = DroneState::STOPPED;
        }
        deltaPath = (prevSpeed + speed) / 2.0f * config.simTimeStep;
      }
      else {
        if (std::fabs(deltaAngle) <= config.turnThreshold)
          direction = desiredDirection;
        float prevSpeed = speed;
        speed += acceleration * config.simTimeStep;
        if (speed >= config.attackSpeed) {
          speed = config.attackSpeed;
          state = DroneState::MOVING;
        }
        deltaPath = (prevSpeed + speed) / 2.0f * config.simTimeStep;
      }
      break;
    case DroneState::DECELERATING: {
      float prevSpeed = speed;
      speed -= acceleration * config.simTimeStep;
      if (speed <= 0) {
        speed = 0;
        state = DroneState::STOPPED;
      }
      deltaPath = (prevSpeed + speed) / 2.0f * config.simTimeStep;
    } break;
    case DroneState::TURNING: {
      float da = Math::normalizeAngle(desiredDirection - direction);
      if (std::fabs(da) <= config.angularSpeed * config.simTimeStep) {
        direction = desiredDirection;
        state = DroneState::ACCELERATING;
      }
      else {
        direction += (da > 0 ? 1.0f : -1.0f) * config.angularSpeed * config.simTimeStep;
        direction = Math::normalizeAngle(direction);
      }
    } break;
    case DroneState::MOVING:
      if (std::fabs(deltaAngle) > config.turnThreshold) {
        state = DroneState::DECELERATING;
        float prevSpeed = speed;
        speed -= acceleration * config.simTimeStep;
        if (speed <= 0) {
          speed = 0;
          state = DroneState::STOPPED;
        }
        deltaPath = (prevSpeed + speed) / 2.0f * config.simTimeStep;
      }
      else {
        if (std::fabs(deltaAngle) <= config.turnThreshold)
          direction = desiredDirection;
        deltaPath = speed * config.simTimeStep;
      }
      break;
  }

  dronePos.x += std::cos(direction) * deltaPath;
  dronePos.y += std::sin(direction) * deltaPath;

  if (state == DroneState::MOVING && Math::length(dronePos - firePoint) <= config.hitRadius * 0.25f) {
    isMissionFinished = true;
    LOG("Target captured. Weapon released at step: " << totalSteps);
  }

  currentTime += config.simTimeStep;
  SimStep currentStepData = steps[totalSteps];
  totalSteps++;
  return currentStepData;
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

void MissionProcessor::changeSolver(IBallisticSolver* s)
{
  if (s)
    solver = s;
}

int MissionProcessor::getTotalSteps() const { return totalSteps; }

const SimStep* MissionProcessor::getStepsHistory() const { return steps; }

Coord MissionProcessor::interpolateTarget(int targetIdx, float t)
{
  int stepsCount = provider->getTimeSteps();
  int idx = (int)std::floor(t / config.arrayTimeStep) % stepsCount;
  int next = (idx + 1) % stepsCount;
  float frac = (t / config.arrayTimeStep) - std::floor(t / config.arrayTimeStep);

  Coord pIdx = provider->getTargetPos(targetIdx, idx);
  Coord pNext = provider->getTargetPos(targetIdx, next);

  return pIdx + (pNext - pIdx) * frac;
}

Coord MissionProcessor::extrapolateTarget(int targetIdx, float time, float dt)
{
  int stepsCount = provider->getTimeSteps();
  int idx = (int)std::floor(time / config.arrayTimeStep) % stepsCount;
  int next = (idx + 1) % stepsCount;

  Coord pIdx = provider->getTargetPos(targetIdx, idx);
  Coord pNext = provider->getTargetPos(targetIdx, next);

  Coord v = (pNext - pIdx) / config.arrayTimeStep;
  Coord curPos = interpolateTarget(targetIdx, time);

  return curPos + v * dt;
}

}  // namespace BallisticApp

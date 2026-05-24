#include "ballistic_app/MissionProcessor.h"
#include "ballistic_app/utils/MathUtils.h"
#include "ballistic_app/Defines.h"
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
  , m_physicsEngine(nullptr)
  , m_targetPredictor(nullptr)
  , m_planner(nullptr)
  , m_steps(nullptr)
  , m_dronePos{0, 0}
  , m_direction(0.0f)
  , m_speed(0.0f)
  , m_state(DroneState::STOPPED)
  , m_currentTime(0.0f)
  , m_totalSteps(0)
  , m_isMissionFinished(false)
  , m_cachedFlightTime(0.0f)
  , m_cachedHDist(0.0f)
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
  if (m_physicsEngine) {
    delete m_physicsEngine;
    m_physicsEngine = nullptr;
  }
  if (m_targetPredictor) {
    delete m_targetPredictor;
    m_targetPredictor = nullptr;
  }
  if (m_planner) {
    delete m_planner;
    m_planner = nullptr;
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
  m_steps = new SimStep[MissionProcessor::MAX_STEPS];
  reset();

  if (m_solver) {
    m_cachedFlightTime = m_solver->calcTimeOfFall(m_config.altitude, m_config.attackSpeed, m_ammo);
    m_cachedHDist = m_solver->calcHDistance(m_cachedFlightTime, m_config.attackSpeed, m_ammo);
  }

  // Перестворення об'єктів під нову конфігурацію конфігу та провайдера
  if (m_physicsEngine)
    delete m_physicsEngine;
  if (m_targetPredictor)
    delete m_targetPredictor;
  if (m_planner)
    delete m_planner;

  m_physicsEngine = new DronePhysicsEngine(m_config);
  m_targetPredictor = new TargetPredictor(m_provider, m_config);
  m_planner = new MissionPlanner(m_physicsEngine, m_targetPredictor, m_config);

  LOG("Mission initialized. Ammo: " + m_ammo.name);
}

bool MissionProcessor::hasNext()
{
  return (m_totalSteps < MissionProcessor::MAX_STEPS) && !m_isMissionFinished;
}

SimStep MissionProcessor::step()
{
  float bestTime = 1e9f;
  int bestTarget = 0;
  Coord bestPredicted{0, 0};
  Coord bestFirePoint{0, 0};

  DronePhysicsState currentDroneState{m_dronePos, m_speed, m_direction, m_state};

  for (int tId = 0; tId < m_provider->getTargetCount(); ++tId) {
    Coord currentFirePoint{0, 0};
    Coord currentPredictedTarget{0, 0};

    float predictedTime = m_planner->predictTimeAndPos(
      currentDroneState, m_currentTime, m_cachedFlightTime, m_cachedHDist, tId, currentFirePoint, currentPredictedTarget);
    if (predictedTime < bestTime) {
      bestTime = predictedTime;
      bestTarget = tId;
      bestPredicted = currentPredictedTarget;
      bestFirePoint = currentFirePoint;
    }
  }

  Coord firePoint = bestFirePoint;
  Coord aimPoint = m_dronePos + Coord{std::cos(m_direction), std::sin(m_direction)} * m_cachedHDist;

  m_steps[m_totalSteps].pos = m_dronePos;
  m_steps[m_totalSteps].direction = m_direction;
  m_steps[m_totalSteps].state = m_state;
  m_steps[m_totalSteps].targetIdx = bestTarget;
  m_steps[m_totalSteps].dropPoint = firePoint;
  m_steps[m_totalSteps].aimPoint = aimPoint;
  m_steps[m_totalSteps].predictedTarget = bestPredicted;

  float deltaPath = 0.0f;
  m_physicsEngine->update(currentDroneState, firePoint, m_config.simTimeStep, deltaPath);

  m_dronePos = currentDroneState.pos;
  m_speed = currentDroneState.speed;
  m_direction = currentDroneState.direction;
  m_state = currentDroneState.state;

  if (m_state == DroneState::MOVING && Math::length(m_dronePos - firePoint) <= m_config.hitRadius * 0.25f) {
    m_isMissionFinished = true;
    LOG("Target captured. Weapon released at step: " + std::to_string(m_totalSteps));
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
}  // namespace BallisticApp

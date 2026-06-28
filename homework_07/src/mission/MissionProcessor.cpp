#include "BallisticApp/mission/MissionProcessor.h"
#include "BallisticApp/mission/ComponentFactory.h"
#include "BallisticApp/utils/Logger.h"
#include "BallisticApp/states/DroneStateRegistry.h"
#include "BallisticApp/interfaces/IConfigLoader.h"
#include "BallisticApp/interfaces/ITargetProvider.h"
#include "BallisticApp/interfaces/IBallisticSolver.h"
#include "BallisticApp/interfaces/ISimulationExporter.h"
#include "BallisticApp/ballistics/FireControlComputer.h"
#include <cmath>
#include <stdexcept>

namespace BallisticApp {

namespace {
constexpr DroneStateType INITIAL_DRONE_STATE = DroneStateType::STOPPED;
constexpr int MAX_STEPS = 10000;
}  // namespace

MissionProcessor::MissionProcessor(const LaunchParams& params)
  : m_loader(ComponentFactory::createLoader(ConfigLoaderType::FILE))
  , m_provider(ComponentFactory::createProvider(TargetProviderType::JSON, params.targetsPath))
  , m_solver(ComponentFactory::createSolver(params.solverType))
  , m_exporter(ComponentFactory::createExporter(ExporterType::JSON, params.simulationPath))
  , m_config()
  , m_ammo()
  , m_autopilot()
  , m_extrapolator(nullptr)
  , m_fireControl(nullptr)
  , m_missionCtx{.cfg = m_config}
  , m_tas(0)
{
  if (!m_loader || !m_provider || !m_solver || !m_exporter) {
    throw std::runtime_error("Critical: Factory failed to instantiate core components.");
  }

  m_loader->load(params.configPath, params.ammoPath);
  m_config = m_loader->getConfig();
  m_ammo = m_loader->getAmmoParams();

  m_extrapolator = std::make_unique<TargetExtrapolator>(*m_provider, m_config.arrayTimeStep);
  m_fireControl = std::make_unique<FireControlComputer>(*m_extrapolator, m_autopilot, m_config.simTimeStep);

  m_tas.reset(m_provider->getTargetCount());
  m_tas.setTargetLockEnabled(params.enableTargetLock);

  if (params.solverType == SolverType::TABLE) {
    if (!m_solver->initialize(params.tablePath)) {
      throw std::runtime_error("Critical: Failed to initialize Ballistic Table Solver.");
    }
  }

  reset();
  APP_LOG_MOD("Mission", "{:.<15} READY [AMMO={}]", "PAYLOAD_STATUS", m_ammo.name);
}

MissionProcessor::~MissionProcessor() = default;

bool MissionProcessor::hasNext() const
{
  return (m_totalSteps < MAX_STEPS) && !m_isMissionFinished;
}

SimStep MissionProcessor::step()
{
  m_missionCtx.currentTime = m_currentTime;

  // 1. TARGET LOCK LOGIC & 2. MULTI-TARGET RADAR SCAN
  auto [bestTarget, bestSolution] = m_tas.acquireBestTarget(m_missionCtx, m_extrapolator, m_fireControl);

  Coord bestPredicted;
  Coord firePoint;

  if (bestTarget != -1) {
    bestPredicted = bestSolution.predictedTarget;
    firePoint = bestSolution.firePoint;
  }
  else {
    // FALLBACK: No successful solution found at all
    bestTarget = 0;
    // Extrapolate target 0 to the current moment (flightTime = 0.0f)
    bestPredicted = m_extrapolator->extrapolate(bestTarget, m_currentTime, 0.0f);
    firePoint = bestPredicted;

    APP_LOG_MOD("Mission", "Warning: No fire solution. Flying directly to target 0 at pos: {}", firePoint);
  }

  // Calculation of the aiming point
  Coord dropToTargetDir = {std::cos(m_missionCtx.direction), std::sin(m_missionCtx.direction)};
  const Coord toFireVector = firePoint - m_missionCtx.pos;

  // Optimized using lengthSquared (1e-4f squared is 1e-8f)
  if (toFireVector.lengthSquared() > 1e-8f) {
    dropToTargetDir = toFireVector.normalize();
  }
  const Coord aimPoint = firePoint + dropToTargetDir * m_missionCtx.hDistance;

  // Form the step history
  SimStep currentStep;
  currentStep.pos = m_missionCtx.pos;
  currentStep.direction = m_missionCtx.direction;
  currentStep.state = m_missionCtx.getCurrentStateType();
  currentStep.targetIdx = bestTarget;
  currentStep.dropPoint = firePoint;
  currentStep.aimPoint = aimPoint;
  currentStep.predictedTarget = bestPredicted;

  // Pass control to the real drone's autopilot
  m_missionCtx.firePoint = firePoint;
  m_autopilot.update(m_missionCtx);

  m_currentTime += m_config.simTimeStep;
  m_steps.push_back(currentStep);
  m_totalSteps++;

  if (m_missionCtx.isTargetCaptured()) {
    m_isMissionFinished = true;
    APP_LOG_MOD("Mission", "{:.<15} TARGET={:0>2} STEP={}", "WEAPON_RELEASE", bestTarget, m_totalSteps);
  }

  return currentStep;
}

void MissionProcessor::run()
{
  while (this->hasNext()) {
    this->step();
  }
  m_exporter->exportSimulation(m_steps);
}

void MissionProcessor::reset()
{
  m_missionCtx.pos = m_config.startPos;
  m_missionCtx.direction = m_config.initialDir;
  m_missionCtx.desiredDir = m_config.initialDir;
  m_missionCtx.speed = 0.0f;
  m_missionCtx.lastDeltaPath = 0.0f;
  m_missionCtx.firePoint = Coord{0.0f, 0.0f};
  m_missionCtx.currentTime = 0.0f;
  m_missionCtx.currentState = DroneStateRegistry::getState(INITIAL_DRONE_STATE);

  updateBallisticCache();

  if (m_provider) {
    m_tas.reset(m_provider->getTargetCount());
  }

  m_currentTime = 0.0f;
  m_totalSteps = 0;
  m_isMissionFinished = false;

  m_steps.clear();
  m_steps.reserve(MAX_STEPS);
}

void MissionProcessor::changeSolver(std::unique_ptr<IBallisticSolver> solver)
{
  if (solver) {
    m_solver = std::move(solver);
    updateBallisticCache();
  }
}

void MissionProcessor::updateBallisticCache()
{
  if (m_solver) {
    auto ballisticResult = m_solver->calculate(m_config.altitude, m_config.attackSpeed, m_ammo);
    m_missionCtx.flightTime = ballisticResult.flightTime;
    m_missionCtx.hDistance = ballisticResult.hDistance;
  }
}

int MissionProcessor::getTotalSteps() const
{
  return m_totalSteps;
}

const std::vector<SimStep>& MissionProcessor::getStepsHistory() const
{
  return m_steps;
}

}  // namespace BallisticApp

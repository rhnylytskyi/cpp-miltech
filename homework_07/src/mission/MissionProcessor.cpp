#include "BallisticApp/mission/MissionProcessor.h"
#include "BallisticApp/mission/ComponentFactory.h"
#include "BallisticApp/interfaces/IConfigLoader.h"
#include "BallisticApp/interfaces/ITargetProvider.h"
#include "BallisticApp/interfaces/IBallisticSolver.h"
#include "BallisticApp/interfaces/ISimulationExporter.h"
#include "BallisticApp/navigation/DroneAutopilot.h"
#include "BallisticApp/states/DroneStateRegistry.h"
#include "BallisticApp/ballistics/TargetExtrapolator.h"
#include "BallisticApp/ballistics/FireControlComputer.h"
#include "BallisticApp/utils/Logger.h"
#include <cmath>
#include <memory>

namespace BallisticApp {

namespace {
constexpr DroneStateType INITIAL_DRONE_STATE = DroneStateType::STOPPED;
constexpr int MAX_STEPS = 10000;
}  // namespace

MissionProcessor::MissionProcessor(const std::filesystem::path& configSource,
                                   const std::filesystem::path& targetsPath,
                                   const std::filesystem::path& ammoSource,
                                   const std::filesystem::path& simulationPath,
                                   bool targetLockEnabled)
  : m_loader(ComponentFactory::createLoader(ConfigLoaderType::FILE))
  , m_provider(ComponentFactory::createProvider(TargetProviderType::JSON, targetsPath))
  , m_solver(ComponentFactory::createSolver(SolverType::ANALYTICAL))
  , m_exporter(ComponentFactory::createExporter(ExporterType::JSON, simulationPath))
  , m_config([this, &configSource, &ammoSource]() {
    if (m_loader)
      m_loader->load(configSource, ammoSource);
    return m_loader ? m_loader->getConfig() : DroneConfig{};
  }())
  , m_ammo(m_loader ? m_loader->getAmmoParams() : AmmoParams{})
  , m_autopilot()
  , m_extrapolator(*m_provider, m_config.arrayTimeStep)
  , m_fireControl(std::make_unique<FireControlComputer>(m_extrapolator, m_autopilot, m_config.simTimeStep))
  , m_missionCtx{.cfg = m_config}
  , m_tas(m_provider ? m_provider->getTargetCount() : 0)
  , m_currentTime(0.0f)
  , m_totalSteps(0)
  , m_isMissionFinished(false)
{
  m_tas.setTargetLockEnabled(targetLockEnabled);
  reset();
  APP_LOG("Mission initialized. Ammo: {}", m_ammo.name);
}

MissionProcessor::~MissionProcessor() = default;

bool MissionProcessor::hasNext()
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
    bestPredicted = m_extrapolator.extrapolate(bestTarget, m_currentTime, 0.0f);
    firePoint = bestPredicted;

    APP_LOG("Warning: No fire solution. Flying directly to target 0 at pos: {}", firePoint);
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
    APP_LOG("Bomb released on target {} at step: {}", bestTarget, m_totalSteps);
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
    const float flightTime = m_solver->calcTimeOfFall(m_config.altitude, m_config.attackSpeed, m_ammo);

    m_missionCtx.flightTime = flightTime;
    m_missionCtx.hDistance = m_solver->calcHDistance(flightTime, m_config.attackSpeed, m_ammo);
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

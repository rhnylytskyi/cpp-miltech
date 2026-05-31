#include "BallisticApp/MissionProcessor.h"
#include "BallisticApp/ComponentFactory.h"
#include "BallisticApp/interfaces/IConfigLoader.h"
#include "BallisticApp/interfaces/ITargetProvider.h"
#include "BallisticApp/interfaces/IBallisticSolver.h"
#include "BallisticApp/interfaces/ISimulationExporter.h"
#include "BallisticApp/states/DroneStateRegistry.h"
#include "BallisticApp/utils/Logger.h"
#include "BallisticApp/DronePhysicsEngine.h"
#include "BallisticApp/TargetPredictor.h"
#include "BallisticApp/FireControlComputer.h"
#include <cmath>

namespace BallisticApp {

namespace {
constexpr DroneStateType INITIAL_DRONE_STATE = DroneStateType::STOPPED;
constexpr int MAX_STEPS = 10000;
}  // namespace

MissionProcessor::MissionProcessor(const std::filesystem::path& configSource,
                                   const std::filesystem::path& targetsPath,
                                   const std::filesystem::path& ammoSource,
                                   const std::filesystem::path& simulationPath)
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
  , m_physicsEngine(std::make_unique<DronePhysicsEngine>())
  , m_targetPredictor(std::make_unique<TargetPredictor>(*m_provider, m_config.arrayTimeStep))
  , m_fireControl(std::make_unique<FireControlComputer>(*m_physicsEngine, *m_targetPredictor, m_config.simTimeStep))
  , m_missionCtx{.cfg = m_config}
  , m_currentTime(0.0f)
  , m_totalSteps(0)
  , m_isMissionFinished(false)
{
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
  const int targetCount = m_provider->getTargetCount();
  m_missionCtx.currentTime = m_currentTime;

  FireSolution bestSolution;
  bestSolution.isSuccess = false;
  bestSolution.time = std::numeric_limits<float>::max();
  int bestTarget = -1;

  for (int tId = 0; tId < targetCount; ++tId) {
    FireSolution sol = m_fireControl->calculateSolution(m_missionCtx, tId);

    if (sol.isSuccess && sol.time < bestSolution.time) {
      bestSolution = sol;
      bestTarget = tId;
    }
  }

  Coord bestPredicted;
  Coord firePoint;

  if (bestTarget != -1) {
    bestPredicted = bestSolution.predictedTarget;
    firePoint = bestSolution.firePoint;
  }
  else {
    // РЕЗЕРВНИЙ ВАРІАНТ: Жодного успішного рішення не знайдено взагалі
    bestTarget = 0;
    // Екстраполюємо ціль 0 на поточний момент (flightTime = 0.0f)
    bestPredicted = m_targetPredictor->extrapolate(bestTarget, m_currentTime, 0.0f);
    firePoint = bestPredicted;

    APP_LOG("Warning: No fire solution. Flying directly to target 0 at pos: {}", firePoint);
  }

  // Розрахунок точки прицілювання
  Coord dropToTargetDir = {std::cos(m_missionCtx.direction), std::sin(m_missionCtx.direction)};
  const float distToFire = (firePoint - m_missionCtx.pos).length();
  if (distToFire > 1e-4f) {
    dropToTargetDir = (firePoint - m_missionCtx.pos).normalize();
  }
  const Coord aimPoint = firePoint + dropToTargetDir * m_missionCtx.hDistance;

  // Формуємо історію кроку
  SimStep currentStep;
  currentStep.pos = m_missionCtx.pos;
  currentStep.direction = m_missionCtx.direction;
  currentStep.state = m_missionCtx.getCurrentStateType();
  currentStep.targetIdx = bestTarget;
  currentStep.dropPoint = firePoint;
  currentStep.aimPoint = aimPoint;
  currentStep.predictedTarget = bestPredicted;

  // Фізика та оновлення часу
  m_missionCtx.firePoint = firePoint;
  m_physicsEngine->update(m_missionCtx);

  m_currentTime += m_config.simTimeStep;
  m_steps.push_back(currentStep);
  m_totalSteps++;

  if (m_missionCtx.isTargetCaptured()) {
    m_isMissionFinished = true;
    APP_LOG("Target captured. Bomb released at step: {}", m_totalSteps);
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

#include "BallisticApp/MissionProcessor.h"
#include "BallisticApp/config/ComponentFactory.h"
#include "BallisticApp/interfaces/IConfigLoader.h"
#include "BallisticApp/interfaces/ITargetProvider.h"
#include "BallisticApp/interfaces/IBallisticSolver.h"
#include "BallisticApp/interfaces/ISimulationExporter.h"
#include "BallisticApp/utils/MathUtils.h"
#include "BallisticApp/states/StateStopped.h"
#include "BallisticApp/Defines.h"
#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace BallisticApp {

namespace {
DroneConfig loadConfigHelper(IConfigLoader* loader, const std::string& cfg, const std::string& ammo)
{
  if (loader) {
    loader->load(cfg, ammo);
    return loader->getConfig();
  }
  return DroneConfig{};
}
}  // namespace

MissionProcessor::MissionProcessor(const std::string& targetsPath,
                                   const std::string& simulationPath,
                                   const std::string& configSource,
                                   const std::string& ammoSource)
  : m_loader(ComponentFactory::createLoader(ConfigLoaderType::FILE))
  , m_provider(ComponentFactory::createProvider(TargetProviderType::JSON, targetsPath))
  , m_solver(ComponentFactory::createSolver(SolverType::ANALYTICAL))
  , m_exporter(ComponentFactory::createExporter(ExporterType::JSON, simulationPath))
  , m_config(loadConfigHelper(m_loader.get(), configSource, ammoSource))
  , m_ammo(m_loader ? m_loader->getAmmoParams() : AmmoParams{})
  , m_physicsEngine(m_config)
  , m_targetPredictor(m_provider.get(), m_config)
  , m_planner(m_physicsEngine, m_targetPredictor, m_config)
  , m_missionCtx{m_config.startPos, 0.0f, m_config.initialDir, 0.0f, 0.0f, m_config}
  , m_currentState(std::make_unique<StateStopped>())
  , m_currentTime(0.0f)
  , m_totalSteps(0)
  , m_isMissionFinished(false)
  , m_cachedFlightTime(0.0f)
  , m_cachedHDist(0.0f)
{
  if (!m_loader || !m_provider || !m_solver || !m_exporter) {
    throw std::runtime_error("MissionProcessor Error: Factory failed to create critical components.");
  }
  m_cachedFlightTime = m_solver->calcTimeOfFall(m_config.altitude, m_config.attackSpeed, m_ammo);
  m_cachedHDist = m_solver->calcHDistance(m_cachedFlightTime, m_config.attackSpeed, m_ammo);

  reset();
  LOG("Mission initialized. Ammo: " + m_ammo.name);
}

MissionProcessor::~MissionProcessor() = default;

bool MissionProcessor::hasNext()
{
  return (m_totalSteps < MissionProcessor::MAX_STEPS) && !m_isMissionFinished;
}

SimStep MissionProcessor::step()
{
  const int targetCount = m_provider->getTargetCount();

  std::vector<TargetCandidate> candidates;
  candidates.reserve(targetCount);

  for (int tId = 0; tId < targetCount; ++tId) {
    TargetCandidate c;
    c.id = tId;
    c.time = m_planner.predictTimeAndPos(
      m_missionCtx, m_currentState->getType(), m_currentTime, m_cachedFlightTime, m_cachedHDist, tId, c.firePoint, c.predictedTarget);
    candidates.push_back(c);
  }

  auto bestIt = std::min_element(
    candidates.begin(), candidates.end(), [](const TargetCandidate& a, const TargetCandidate& b) { return a.time < b.time; });

  const int bestTarget = bestIt->id;
  const Coord bestPredicted = bestIt->predictedTarget;
  const Coord firePoint = bestIt->firePoint;

  // Розрахунок точки прицілювання
  Coord dropToTargetDir = {std::cos(m_missionCtx.direction), std::sin(m_missionCtx.direction)};
  const float distToFire = Math::length(firePoint - m_missionCtx.pos);
  if (distToFire > 1e-4f) {
    dropToTargetDir = Math::normalize(firePoint - m_missionCtx.pos);
  }
  const Coord aimPoint = firePoint + dropToTargetDir * m_cachedHDist;

  // Зберігаємо поточний стан до оновлення фізики для формування логу/історії кроку
  SimStep currentStep;
  currentStep.pos = m_missionCtx.pos;
  currentStep.direction = m_missionCtx.direction;
  currentStep.state = m_currentState->getType();
  currentStep.targetIdx = bestTarget;
  currentStep.dropPoint = firePoint;
  currentStep.aimPoint = aimPoint;
  currentStep.predictedTarget = bestPredicted;

  // Крок фізичного двигуна (контекст і стан модифікуються всередині)
  m_physicsEngine.update(m_missionCtx, m_currentState, firePoint);

  // Перевірка умови скидання боєприпасу
  if (m_missionCtx.isTargetCaptured(m_currentState->getType(), firePoint)) {
    m_isMissionFinished = true;
    LOG("Target captured. Bomb released at step: " + std::to_string(m_totalSteps));
  }

  m_currentTime += m_config.simTimeStep;
  m_steps.push_back(currentStep);
  m_totalSteps++;

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
  m_missionCtx.speed = 0.0f;
  m_missionCtx.desiredDir = m_config.initialDir;
  m_missionCtx.lastDeltaPath = 0.0f;

  m_currentState = std::make_unique<StateStopped>();
  m_currentTime = 0.0f;
  m_totalSteps = 0;
  m_isMissionFinished = false;
  m_steps.clear();
  m_steps.reserve(MissionProcessor::MAX_STEPS);
}

void MissionProcessor::changeSolver(std::unique_ptr<IBallisticSolver> solver)
{
  if (solver) {
    m_solver = std::move(solver);
    m_cachedFlightTime = m_solver->calcTimeOfFall(m_config.altitude, m_config.attackSpeed, m_ammo);
    m_cachedHDist = m_solver->calcHDistance(m_cachedFlightTime, m_config.attackSpeed, m_ammo);
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

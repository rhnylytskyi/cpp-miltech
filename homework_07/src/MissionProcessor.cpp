#include "BallisticApp/MissionProcessor.h"
#include "BallisticApp/ComponentFactory.h"
#include "BallisticApp/interfaces/IConfigLoader.h"
#include "BallisticApp/interfaces/ITargetProvider.h"
#include "BallisticApp/interfaces/IBallisticSolver.h"
#include "BallisticApp/interfaces/ISimulationExporter.h"
#include "BallisticApp/utils/MathUtils.h"
#include "BallisticApp/Defines.h"
#include <cmath>
#include <algorithm>

namespace BallisticApp {

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
  , m_physicsEngine(std::make_unique<DronePhysicsEngine>(m_config))
  , m_targetPredictor(std::make_unique<TargetPredictor>(*m_provider, m_config))
  , m_fireControl(std::make_unique<FireControlComputer>(*m_physicsEngine, *m_targetPredictor, m_config))
  , m_missionCtx([this]() {
    const float flightTime = m_solver ? m_solver->calcTimeOfFall(m_config.altitude, m_config.attackSpeed, m_ammo) : 0.0f;
    const float hDistance = m_solver ? m_solver->calcHDistance(flightTime, m_config.attackSpeed, m_ammo) : 0.0f;

    return MissionContext{.pos = m_config.startPos,
                          .speed = 0.0f,
                          .direction = m_config.initialDir,
                          .desiredDir = m_config.initialDir,
                          .lastDeltaPath = 0.0f,
                          .cfg = m_config,
                          .currentState = ComponentFactory::getState(DroneStateType::STOPPED),
                          .currentStateType = DroneStateType::STOPPED,
                          .firePoint = Coord{0.0f, 0.0f},
                          .currentTime = 0.0f,
                          .cachedFlightTime = flightTime,
                          .cachedHDist = hDistance};
  }())
  , m_currentTime(0.0f)
  , m_totalSteps(0)
  , m_isMissionFinished(false)
{
  reset();
  LOG("Mission initialized. Ammo: " + m_ammo.name);
}

MissionProcessor::~MissionProcessor() = default;

bool MissionProcessor::hasNext()
{
  return (m_totalSteps < MAX_STEPS) && !m_isMissionFinished;
}

SimStep MissionProcessor::step()
{
  const int targetCount = m_provider->getTargetCount();

  m_candidates.clear();

  m_missionCtx.currentTime = m_currentTime;

  for (int tId = 0; tId < targetCount; ++tId) {
    TargetCandidate c;
    c.id = tId;
    c.solution = m_fireControl->calculateSolution(m_missionCtx, tId);
    m_candidates.push_back(c);
  }

  // Знаходимо кандидата з мінімальним часом польоту до точки скидання
  auto bestIt = std::min_element(m_candidates.begin(), m_candidates.end(), [](const TargetCandidate& a, const TargetCandidate& b) {
    return a.solution.time < b.solution.time;
  });

  int bestTarget = bestIt->id;
  Coord bestPredicted = bestIt->solution.predictedTarget;
  Coord firePoint = bestIt->solution.firePoint;

  // РЕЗЕРВНИЙ ВАРІАНТ: Якщо математичне рішення для обраної цілі НЕ успішне
  if (!bestIt->solution.isSuccess && !m_candidates.empty()) {
    // Беремо найпершу ціль (індекс 0)
    bestTarget = 0;

    // Запитуємо у предиктора, де ця ціль знаходиться прямо ЗАРАЗ (в поточний момент часу)
    // Для цього передаємо cachedFlightTime = 0.0f
    bestPredicted = m_targetPredictor->extrapolate(bestTarget, m_currentTime, 0.0f);

    // Точкою вогню стає сама позиція цієї цілі (летимо просто на неї)
    firePoint = bestPredicted;

    LOG("Warning: No fire solution. Flying directly to target 0 at pos: {" + std::to_string(firePoint.x) + ", " +
        std::to_string(firePoint.y) + "}");
  }

  // Розрахунок точки прицілювання
  Coord dropToTargetDir = {std::cos(m_missionCtx.direction), std::sin(m_missionCtx.direction)};
  const float distToFire = Math::length(firePoint - m_missionCtx.pos);
  if (distToFire > 1e-4f) {
    dropToTargetDir = Math::normalize(firePoint - m_missionCtx.pos);
  }
  const Coord aimPoint = firePoint + dropToTargetDir * m_missionCtx.cachedHDist;

  // Формуємо історію кроку
  SimStep currentStep;
  currentStep.pos = m_missionCtx.pos;
  currentStep.direction = m_missionCtx.direction;
  currentStep.state = m_missionCtx.currentStateType;
  currentStep.targetIdx = bestTarget;
  currentStep.dropPoint = firePoint;
  currentStep.aimPoint = aimPoint;
  currentStep.predictedTarget = bestPredicted;

  // Фізичний рушій
  m_missionCtx.firePoint = firePoint;
  m_physicsEngine->update(m_missionCtx);

  if (m_missionCtx.isTargetCaptured()) {
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

  m_missionCtx.currentState = ComponentFactory::getState(DroneStateType::STOPPED);
  m_missionCtx.currentStateType = DroneStateType::STOPPED;
  m_missionCtx.firePoint = Coord{0.0f, 0.0f};
  m_missionCtx.currentTime = 0.0f;

  m_currentTime = 0.0f;
  m_totalSteps = 0;
  m_isMissionFinished = false;
  m_steps.clear();
  m_steps.reserve(MAX_STEPS);

  m_candidates.clear();
  if (m_provider) {
    m_candidates.reserve(m_provider->getTargetCount());
  }
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

    m_missionCtx.cachedFlightTime = flightTime;
    m_missionCtx.cachedHDist = m_solver->calcHDistance(flightTime, m_config.attackSpeed, m_ammo);
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

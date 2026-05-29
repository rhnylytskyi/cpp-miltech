#include "ballistic_app/MissionProcessor.h"
#include "ballistic_app/config/ComponentFactory.h"
#include "ballistic_app/interfaces/IConfigLoader.h"
#include "ballistic_app/interfaces/ITargetProvider.h"
#include "ballistic_app/interfaces/IBallisticSolver.h"
#include "ballistic_app/interfaces/ISimulationExporter.h"
#include "ballistic_app/utils/MathUtils.h"
#include "ballistic_app/Defines.h"
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
  const DronePhysicsState currentDroneState{m_dronePos, m_speed, m_direction, m_state};
  const int targetCount = m_provider->getTargetCount();

  // Пошук найкращої цілі через віртуальний прогноз польоту
  std::vector<TargetCandidate> candidates;
  candidates.reserve(targetCount);

  for (int tId = 0; tId < targetCount; ++tId) {
    TargetCandidate c;
    c.id = tId;
    c.time =
      m_planner.predictTimeAndPos(currentDroneState, m_currentTime, m_cachedFlightTime, m_cachedHDist, tId, c.firePoint, c.predictedTarget);
    candidates.push_back(c);
  }

  auto bestIt = std::min_element(
    candidates.begin(), candidates.end(), [](const TargetCandidate& a, const TargetCandidate& b) { return a.time < b.time; });

  const int bestTarget = bestIt->id;
  const Coord bestPredicted = bestIt->predictedTarget;
  const Coord bestFirePoint = bestIt->firePoint;

  // Розрахунок параметрів скидання на основі знайденої найкращої цілі
  const Coord firePoint = bestFirePoint;

  // Розрахунок точного вектора прицілювання (aimPoint) попереду точки скидання
  Coord dropToTargetDir = {std::cos(m_direction), std::sin(m_direction)};
  const float distToFire = Math::length(firePoint - m_dronePos);
  if (distToFire > 1e-4f) {
    dropToTargetDir = Math::normalize(firePoint - m_dronePos);
  }
  const Coord aimPoint = firePoint + dropToTargetDir * m_cachedHDist;

  // Запис поточного кроку в історію симуляції
  SimStep currentStep;
  currentStep.pos = m_dronePos;
  currentStep.direction = m_direction;
  currentStep.state = m_state;
  currentStep.targetIdx = bestTarget;
  currentStep.dropPoint = firePoint;
  currentStep.aimPoint = aimPoint;
  currentStep.predictedTarget = bestPredicted;

  // Оновлення реальних фізичних параметрів та позиції дрона
  float deltaPath = 0.0f;
  DronePhysicsState updatedState = currentDroneState;
  m_physicsEngine.update(updatedState, firePoint, m_config.simTimeStep, deltaPath);

  m_dronePos = updatedState.pos;
  m_speed = updatedState.speed;
  m_direction = updatedState.direction;
  m_state = updatedState.state;

  // Перевірка умови завершення місії
  if (m_state == DroneState::MOVING && Math::length(m_dronePos - firePoint) <= m_config.hitRadius * 0.25f) {
    m_isMissionFinished = true;
    LOG("Target captured. Weapon released at step: " + std::to_string(m_totalSteps));
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
  m_dronePos = m_config.startPos;
  m_direction = m_config.initialDir;
  m_speed = 0.0f;
  m_state = DroneState::STOPPED;
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

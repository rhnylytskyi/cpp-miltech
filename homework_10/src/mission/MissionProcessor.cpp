#include "BallisticApp/mission/MissionProcessor.h"
#include "BallisticApp/mission/ComponentFactory.h"
#include "BallisticApp/utils/Logger.h"
#include "BallisticApp/mission/MissionContext.h"
#include "BallisticApp/states/DroneStateRegistry.h"
#include "BallisticApp/navigation/DroneCommand.h"
#include "BallisticApp/utils/MathUtils.h"
#include "BallisticApp/mission/AppArguments.h"
#include <cmath>
#include <stdexcept>
#include <thread>
#include <chrono>
#include <algorithm>

namespace BallisticApp {
namespace {
constexpr int MAX_STEPS = 10000;
constexpr float OVERLONG = 1e9f;
}  // namespace

MissionProcessor::MissionProcessor(const AppArguments& appArgs, ThreadSafeTargetProvider& provider, DronePhysics& physics)
  : m_provider(provider)
  , m_physics(physics)
  , m_loader(ComponentFactory::createLoader(ConfigLoaderType::FILE))
  , m_solver(ComponentFactory::createSolver(appArgs.getSolverType()))
  , m_exporter(ComponentFactory::createExporter(ExporterType::JSON, appArgs.getSimulationPath()))
  , m_enableTargetLockCLI(appArgs.isTargetLockEnabled())
{
  if (!m_loader || !m_solver || !m_exporter) {
    throw std::runtime_error("Critical: Initialization of core factory components failed.");
  }

  m_loader->load(appArgs.getConfigPath(), appArgs.getAmmoPath());
  m_config = m_loader->getConfig();
  m_ammo = m_loader->getAmmoParams();

  m_physics.configure(m_config);
  m_provider.setTiming(ThreadSafeTargetProvider::FloatSeconds{m_config.arrayTimeStep}, m_config.timeScale);

  if (appArgs.getSolverType() == SolverType::TABLE) {
    if (!m_solver->initialize(appArgs.getTablePath())) {
      throw std::runtime_error("Critical: Failed to initialize Ballistic Table Solver.");
    }
  }

  reset();
  APP_LOG_MOD("Mission", "{:.<15} READY [AMMO={}, LOCK_CLI={}]", "PAYLOAD_STATUS", m_ammo.name, m_enableTargetLockCLI ? "ON" : "OFF");
}

MissionProcessor::~MissionProcessor()
{
  stop();
}

bool MissionProcessor::isThreadReady() const
{
  return m_isReady.load();
}

void MissionProcessor::start()
{
  m_isStarted.store(true);
}

void MissionProcessor::stop()
{
  m_stopRequested.store(true);
}

bool MissionProcessor::hasNext() const
{
  std::lock_guard<std::mutex> lock(m_stepsMutex);
  return (m_totalSteps < MAX_STEPS) && !m_isMissionFinished && !m_stopRequested.load();
}

SimStep MissionProcessor::step()
{
  DroneTelemetry telemetry = m_physics.waitTelemetry(m_telemetrySeqNum);
  float currentSimTime = telemetry.timeSecSinceStart;

  MissionContext currentCtx(m_config);
  currentCtx.pos = telemetry.pos;
  currentCtx.direction = telemetry.direction;
  currentCtx.speed = telemetry.speed;
  currentCtx.currentTime = currentSimTime;
  currentCtx.flightTime = m_cachedFlightTime;
  currentCtx.hDistance = m_cachedHDistance;
  currentCtx.currentState = DroneStateRegistry::getState(telemetry.state);

  int targetCount = m_provider.getTargetCount();
  float bestTotalCost = OVERLONG;
  int bestTargetId = -1;

  Coord bestFirePoint{0.0f, 0.0f};
  Coord bestAimPoint{0.0f, 0.0f};
  Coord bestPredictedTarget{0.0f, 0.0f};

  float droneAcceleration = 0.0f;
  if (m_config.accelPath > 1e-5f) {
    droneAcceleration = (m_config.attackSpeed * m_config.attackSpeed) / (3.0f * m_config.accelPath);
  }

  int startIdx = 0;
  int endIdx = targetCount;

  if (m_enableTargetLockCLI && m_lockedTargetId != -1) {
    startIdx = m_lockedTargetId;
    endIdx = m_lockedTargetId + 1;
  }
  for (int i = startIdx; i < endIdx; ++i) {
    float t_approximation = m_cachedFlightTime;
    Coord predictedTgt;

    for (int iter = 0; iter < 8; ++iter) {
      Target futureTgt = m_provider.getTargetAtFutureTime(i, t_approximation, currentSimTime + m_config.simTimeStep);
      predictedTgt = futureTgt.pos;

      float travelDist = std::max(0.0f, (predictedTgt - telemetry.pos).length() - m_cachedHDistance);
      t_approximation = travelDist / m_config.attackSpeed + m_cachedFlightTime;
    }

    Coord toTargetVec = predictedTgt - telemetry.pos;
    Coord currentFirePoint = predictedTgt;
    if (toTargetVec.lengthSquared() > 1e-8f) {
      currentFirePoint = predictedTgt - toTargetVec.normalize() * m_cachedHDistance;
    }

    Coord currentAimPoint = currentFirePoint + (currentFirePoint - telemetry.pos).normalize() * m_cachedHDistance;

    float moveT = (currentFirePoint - telemetry.pos).length() / m_config.attackSpeed;
    float targetAngle = std::atan2(currentAimPoint.y - telemetry.pos.y, currentAimPoint.x - telemetry.pos.x);
    float deltaAngle = std::fabs(Math::normalizeAngle(targetAngle - telemetry.direction));
    float turnT = deltaAngle / m_config.angularSpeed;

    float stopT = 0.0f;
    if (m_lockedTargetId != i && droneAcceleration > 1e-5f) {
      if (telemetry.state == DroneStateType::ACCELERATING || telemetry.state == DroneStateType::MOVING) {
        stopT = telemetry.speed / droneAcceleration;
      }
    }

    float distToAim = (currentAimPoint - telemetry.pos).length();
    float arcPenalty = 0.0f;
    if (distToAim < m_cachedHDistance) {
      arcPenalty = 2.0f * (m_cachedHDistance - distToAim) / m_config.attackSpeed + 3.14159f / m_config.angularSpeed;
    }

    float totalCost = moveT + m_cachedFlightTime + turnT + stopT + arcPenalty;

    if (totalCost < bestTotalCost) {
      bestTotalCost = totalCost;
      bestTargetId = i;
      bestFirePoint = currentFirePoint;
      bestAimPoint = currentAimPoint;
      bestPredictedTarget = predictedTgt;
    }
  }

  if (bestTargetId == -1) {
    if (targetCount > 0) {
      bestTargetId = 0;
      Target t = m_provider.getTarget(0);
      bestFirePoint = t.pos;
      bestAimPoint = t.pos;
      bestPredictedTarget = t.pos;
    }
    else {
      bestTargetId = 0;
      bestFirePoint = {0.0f, 0.0f};
      bestAimPoint = {0.0f, 0.0f};
      bestPredictedTarget = {0.0f, 0.0f};
    }
  }

  m_lockedTargetId = bestTargetId;

  float finalTargetAngle = std::atan2(bestFirePoint.y - telemetry.pos.y, bestFirePoint.x - telemetry.pos.x);
  float angleDeviation = Math::normalizeAngle(finalTargetAngle - telemetry.direction);

  constexpr float smoothingFactor = 0.5f;
  float desiredDir = Math::normalizeAngle(telemetry.direction + angleDeviation * smoothingFactor);

  DroneStateType requiredState = telemetry.state;
  float currentDeltaAngle = std::fabs(Math::normalizeAngle(desiredDir - telemetry.direction));

  float finalThresh = std::atan2(m_config.hitRadius * 0.4f, m_cachedHDistance);
  float activeTurnThreshold = std::min(m_config.turnThreshold, finalThresh);

  if (currentDeltaAngle > activeTurnThreshold) {
    if (telemetry.state == DroneStateType::MOVING || telemetry.state == DroneStateType::ACCELERATING) {
      requiredState = DroneStateType::DECELERATING;
    }
    else {
      requiredState = DroneStateType::TURNING;
    }
  }
  else {
    if (telemetry.state == DroneStateType::STOPPED || telemetry.state == DroneStateType::TURNING) {
      requiredState = DroneStateType::ACCELERATING;
    }
    else {
      requiredState = DroneStateType::MOVING;
    }
  }

  DroneCommand cmd{.stateType = requiredState, .desiredDir = desiredDir};
  m_physics.postCommand(cmd);

  SimStep currentStep;
  currentStep.pos = telemetry.pos;
  currentStep.direction = telemetry.direction;
  currentStep.state = requiredState;
  currentStep.targetIdx = bestTargetId;
  currentStep.dropPoint = bestFirePoint;
  currentStep.aimPoint = bestAimPoint;
  currentStep.predictedTarget = bestPredictedTarget;
  currentStep.timeSecSinceStart = currentSimTime;

  {
    std::lock_guard<std::mutex> lock(m_stepsMutex);
    m_steps.push_back(currentStep);
    m_totalSteps++;

    Coord bombLanding = telemetry.pos + Coord{std::cos(telemetry.direction), std::sin(telemetry.direction)} * m_cachedHDistance;
    Target exactFutureTarget = m_provider.getTargetAtFutureTime(bestTargetId, m_cachedFlightTime, currentSimTime + m_config.simTimeStep);

    float bombToTargetDistance = (bombLanding - exactFutureTarget.pos).length();

    if (telemetry.state == DroneStateType::MOVING && currentDeltaAngle <= activeTurnThreshold &&
        bombToTargetDistance <= m_config.hitRadius) {
      m_isMissionFinished = true;
      APP_LOG_MOD(
        "Mission", "{:.<15} TARGET={:0>2} STEP={} DIST={:.2f}", "WEAPON_RELEASE", bestTargetId, m_totalSteps, bombToTargetDistance);
    }

    float currentHitDist = telemetry.pos.distanceTo(bestFirePoint);
    if (m_totalSteps > 1 && currentHitDist > m_prevHitDistance && m_prevHitDistance < m_config.hitRadius * 2.0f) {
      m_isMissionFinished = true;
      APP_LOG_MOD("Mission", "{:.<15} MISSED PAST FIREPOINT. STOPPING.", "SAFETY_ABORT");
    }
    m_prevHitDistance = currentHitDist;
  }

  return currentStep;
}

void MissionProcessor::run()
{
  m_isReady.store(true);
  while (!m_stopRequested.load() && !m_isStarted.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  reset();

  while (hasNext()) {
    step();

    float dt = 0.1f;
    float scale = 1.0f;
    {
      std::lock_guard<std::mutex> lock(m_stepsMutex);
      dt = m_config.simTimeStep;
      scale = m_config.timeScale;
    }
    const float sleepSeconds = dt / std::max(scale, 1e-5f);
    std::this_thread::sleep_for(std::chrono::duration<float>(sleepSeconds));
  }

  std::vector<SimStep> historyCopy = getStepsHistory();
  m_exporter->exportSimulation(historyCopy);
}

void MissionProcessor::reset()
{
  std::lock_guard<std::mutex> lock(m_stepsMutex);
  updateBallisticCache();
  m_lockedTargetId = -1;
  m_totalSteps = 0;
  m_telemetrySeqNum = 0;
  m_prevHitDistance = OVERLONG;
  m_isMissionFinished = false;
  m_steps.clear();
  m_steps.reserve(MAX_STEPS);
  m_physics.reset(m_config.startPos, m_config.initialDir);
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
    m_cachedFlightTime = ballisticResult.flightTime;
    m_cachedHDistance = ballisticResult.hDistance;
  }
}

int MissionProcessor::getTotalSteps() const
{
  std::lock_guard<std::mutex> lock(m_stepsMutex);
  return m_totalSteps;
}

std::vector<SimStep> MissionProcessor::getStepsHistory() const
{
  std::lock_guard<std::mutex> lock(m_stepsMutex);
  return m_steps;
}

}  // namespace BallisticApp

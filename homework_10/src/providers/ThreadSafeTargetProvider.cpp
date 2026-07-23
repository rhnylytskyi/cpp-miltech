#include "BallisticApp/providers/ThreadSafeTargetProvider.h"
#include "BallisticApp/utils/Logger.h"
#include <fstream>
#include <cmath>
#include <thread>
#include <chrono>
#include <algorithm>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace BallisticApp {

ThreadSafeTargetProvider::ThreadSafeTargetProvider(std::filesystem::path filepath)
  : m_filepath(std::move(filepath))
{
}

ThreadSafeTargetProvider::~ThreadSafeTargetProvider()
{
  stop();
}

bool ThreadSafeTargetProvider::load()
{
  std::ifstream f(m_filepath);
  if (!f.is_open()) {
    APP_LOG_MOD("Targets", "Error: Failed to open file: {}", m_filepath.string());
    return false;
  }

  json j_data;
  f >> j_data;
  f.close();

  if (!j_data.contains("targetCount") || !j_data.contains("timeSteps") || !j_data.contains("targets")) {
    APP_LOG_MOD("Targets", "Error: Invalid JSON format. Missing required fields in: {}", m_filepath.string());
    return false;
  }

  int loadedTargetCount = j_data["targetCount"];
  int loadedTimeSteps = j_data["timeSteps"];

  std::vector<std::vector<Coord>> loadedPaths;
  loadedPaths.reserve(static_cast<size_t>(loadedTargetCount));

  for (const auto& jTarget : j_data["targets"]) {
    std::vector<Coord> target_positions;
    target_positions.reserve(static_cast<size_t>(loadedTimeSteps));

    for (const auto& j_pos : jTarget["positions"]) {
      Coord c;
      c.x = j_pos["x"].get<float>();
      c.y = j_pos["y"].get<float>();
      target_positions.push_back(c);
    }
    loadedPaths.push_back(std::move(target_positions));
  }

  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_paths = std::move(loadedPaths);
    m_targetCount = loadedTargetCount;
    m_timeSteps = loadedTimeSteps;
    m_currentTargets.resize(static_cast<size_t>(m_targetCount));
    updateCurrentTargetsLocked(0);
  }

  return true;
}

bool ThreadSafeTargetProvider::isThreadReady() const
{
  return m_isReady.load();
}

void ThreadSafeTargetProvider::start()
{
  m_isStarted.store(true);
}

void ThreadSafeTargetProvider::stop()
{
  m_stopRequested.store(true);
}

int ThreadSafeTargetProvider::getTargetCount() const
{
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_targetCount;
}

int ThreadSafeTargetProvider::getTimeSteps() const
{
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_timeSteps;
}

Target ThreadSafeTargetProvider::getTarget(int targetIdx) const
{
  std::lock_guard<std::mutex> lock(m_mutex);
  if (targetIdx < 0 || targetIdx >= m_targetCount) {
    return Target();
  }
  return m_currentTargets[static_cast<size_t>(targetIdx)];
}

std::vector<Target> ThreadSafeTargetProvider::getSnapshot() const
{
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_currentTargets;
}

void ThreadSafeTargetProvider::setTiming(FloatSeconds arrayTimeStep, float timeScale)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  if (arrayTimeStep > FloatSeconds::zero()) {
    m_arrayTimeStep = arrayTimeStep;
  }
  if (timeScale > 1e-5f) {
    m_timeScale = timeScale;
  }
}

// Calculate velocity as the finite difference between the current and next step
Coord ThreadSafeTargetProvider::calcVelocityLocked(size_t targetIdx, size_t timeIdx) const
{
  if (m_timeSteps <= 1 || m_arrayTimeStep <= FloatSeconds::zero()) {
    return {0.0f, 0.0f};
  }

  size_t n = static_cast<size_t>(m_timeSteps);
  size_t nextIdx = (timeIdx + 1) % n;

  const Coord current = m_paths[targetIdx][timeIdx];
  const Coord next = m_paths[targetIdx][nextIdx];

  return (next - current) / m_arrayTimeStep.count();
}

void ThreadSafeTargetProvider::updateCurrentTargetsLocked(int stepIndex)
{
  if (m_targetCount <= 0 || m_timeSteps <= 0) {
    return;
  }

  size_t wrappedIndex = static_cast<size_t>(stepIndex % m_timeSteps);
  size_t targetCount = static_cast<size_t>(m_targetCount);

  for (size_t i = 0; i < targetCount; ++i) {
    m_currentTargets[i] = {m_paths[i][wrappedIndex], calcVelocityLocked(i, wrappedIndex)};
  }
}

// Linear interpolation of the target's position relative to the clean virtual time
Target ThreadSafeTargetProvider::getTargetAtFutureTime(int targetIdx, float futureSeconds, float offsetSimTime) const
{
  std::lock_guard<std::mutex> lock(m_mutex);

  if (targetIdx < 0 || targetIdx >= m_targetCount || m_timeSteps <= 0) {
    return Target();
  }

  size_t uTargetIdx = static_cast<size_t>(targetIdx);
  size_t n = static_cast<size_t>(m_timeSteps);
  float dt = m_arrayTimeStep.count();

  // Calculate the total virtual time from the start
  float totalSimTime = offsetSimTime + futureSeconds;

  // Use floor for index stability
  size_t futureIdx = static_cast<size_t>(std::floor(totalSimTime / dt)) % n;
  size_t afterIdx = (futureIdx + 1) % n;

  // Calculate velocity on this segment
  Coord vel = (m_paths[uTargetIdx][afterIdx] - m_paths[uTargetIdx][futureIdx]) / dt;

  // Calculate the fractional remainder of the time within the current step
  float fracTime = totalSimTime - std::floor(totalSimTime / dt) * dt;

  // Linear interpolation of the target's position relative to the clean virtual time
  Coord interpolatedPos;
  interpolatedPos.x = m_paths[uTargetIdx][futureIdx].x + vel.x * fracTime;
  interpolatedPos.y = m_paths[uTargetIdx][futureIdx].y + vel.y * fracTime;

  return {interpolatedPos, vel};
}

void ThreadSafeTargetProvider::run()
{
  m_isReady.store(true);
  while (!m_stopRequested.load() && !m_isStarted.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  int stepIndex = 0;

  while (!m_stopRequested.load()) {
    FloatSeconds currentStep;
    float currentScale;
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      updateCurrentTargetsLocked(stepIndex);
      currentStep = m_arrayTimeStep;
      currentScale = m_timeScale;
    }

    ++stepIndex;

    const FloatSeconds sleepDuration = currentStep / std::max(currentScale, MIN_TIME_SCALE);
    std::this_thread::sleep_for(sleepDuration);
  }
}

}  // namespace BallisticApp

#include "BallisticApp/TargetPredictor.h"
#include <cmath>

namespace BallisticApp {

TargetPredictor::TargetPredictor(ITargetProvider* provider, const DroneConfig& config)
  : m_provider(provider)
  , m_config(config)
{
}

Coord TargetPredictor::interpolate(int targetIdx, float t) const
{
  if (!m_provider) {
    return {0, 0};
  }

  const auto stepsCount = m_provider->getTimeSteps();
  if (stepsCount == 0) {
    return {0, 0};
  }

  const float normalizedTime = t / m_config.arrayTimeStep;
  const float floorTime = std::floor(normalizedTime);

  const int idx = static_cast<int>(floorTime) % stepsCount;
  const int next = (idx + 1) % stepsCount;
  const float frac = normalizedTime - floorTime;

  const Coord pIdx = m_provider->getTargetPos(targetIdx, idx);
  const Coord pNext = m_provider->getTargetPos(targetIdx, next);

  return pIdx + (pNext - pIdx) * frac;
}

Coord TargetPredictor::extrapolate(int targetIdx, float time, float dt) const
{
  if (!m_provider) {
    return {0, 0};
  }

  const auto stepsCount = m_provider->getTimeSteps();
  if (stepsCount == 0) {
    return {0, 0};
  }

  const float normalizedTime = time / m_config.arrayTimeStep;
  const int idx = static_cast<int>(std::floor(normalizedTime)) % stepsCount;
  const int next = (idx + 1) % stepsCount;

  const Coord pIdx = m_provider->getTargetPos(targetIdx, idx);
  const Coord pNext = m_provider->getTargetPos(targetIdx, next);

  const Coord v = (pNext - pIdx) / m_config.arrayTimeStep;
  const Coord curPos = interpolate(targetIdx, time);

  return curPos + v * dt;
}

}  // namespace BallisticApp

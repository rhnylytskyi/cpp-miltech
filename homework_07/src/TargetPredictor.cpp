#include "ballistic_app/TargetPredictor.h"
#include <cmath>

namespace BallisticApp {
TargetPredictor::TargetPredictor(ITargetProvider* provider, const DroneConfig& config)
  : m_provider(provider)
  , m_config(config)
{
}

Coord TargetPredictor::interpolate(int targetIdx, float t) const
{
  if (!m_provider)
    return {0, 0};

  int stepsCount = m_provider->getTimeSteps();
  if (stepsCount == 0)
    return {0, 0};

  int idx = (int)std::floor(t / m_config.arrayTimeStep) % stepsCount;
  int next = (idx + 1) % stepsCount;
  float frac = (t / m_config.arrayTimeStep) - std::floor(t / m_config.arrayTimeStep);

  Coord pIdx = m_provider->getTargetPos(targetIdx, idx);
  Coord pNext = m_provider->getTargetPos(targetIdx, next);
  return pIdx + (pNext - pIdx) * frac;
}

Coord TargetPredictor::extrapolate(int targetIdx, float time, float dt) const
{
  if (!m_provider)
    return {0, 0};

  int stepsCount = m_provider->getTimeSteps();
  if (stepsCount == 0)
    return {0, 0};

  int idx = (int)std::floor(time / m_config.arrayTimeStep) % stepsCount;
  int next = (idx + 1) % stepsCount;

  Coord pIdx = m_provider->getTargetPos(targetIdx, idx);
  Coord pNext = m_provider->getTargetPos(targetIdx, next);

  Coord v = (pNext - pIdx) / m_config.arrayTimeStep;
  Coord curPos = interpolate(targetIdx, time);
  return curPos + v * dt;
}
}  // namespace BallisticApp

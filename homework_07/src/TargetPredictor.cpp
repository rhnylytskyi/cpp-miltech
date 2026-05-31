#include "BallisticApp/TargetPredictor.h"
#include <cmath>
#include <algorithm>

namespace BallisticApp {

TargetPredictor::TargetPredictor(const ITargetProvider& provider, float targetTimeStep)
  : m_provider(provider)
  , m_targetTimeStep(targetTimeStep)
{
}

Coord TargetPredictor::interpolate(int targetIdx, float t) const
{
  const auto stepsCount = m_provider.getTimeSteps();
  if (stepsCount <= 0) {
    return {0, 0};
  }

  // ЗАХИСТ: час не може бути від'ємним для індексації
  if (t < 0.0f)
    t = 0.0f;

  const float normalizedTime = t / m_targetTimeStep;
  const float floorTime = std::floor(normalizedTime);

  // ЗАХИСТ ВІД ПЕРЕПОВНЕННЯ: суворо обмежуємо індекси в межах [0, stepsCount - 1]
  int idx = static_cast<int>(floorTime) % stepsCount;
  if (idx < 0)
    idx += stepsCount;  // Гарантуємо, що індекс завжди позитивний!

  int next = (idx + 1) % stepsCount;
  if (next < 0)
    next += stepsCount;

  const float frac = normalizedTime - floorTime;

  const Coord pIdx = m_provider.getTargetPos(targetIdx, idx);
  const Coord pNext = m_provider.getTargetPos(targetIdx, next);

  return pIdx + (pNext - pIdx) * frac;
}

Coord TargetPredictor::extrapolate(int targetIdx, float time, float dt) const
{
  const auto stepsCount = m_provider.getTimeSteps();
  if (stepsCount <= 0) {
    return {0, 0};
  }

  // ЗАХИСТ: клемпінг часу в плюс
  float validTime = std::max(0.0f, time);

  const float normalizedTime = validTime / m_targetTimeStep;

  int idx = static_cast<int>(std::floor(normalizedTime)) % stepsCount;
  if (idx < 0)
    idx += stepsCount;  // Захист від від'ємного за модулем

  int next = (idx + 1) % stepsCount;
  if (next < 0)
    next += stepsCount;

  const Coord pIdx = m_provider.getTargetPos(targetIdx, idx);
  const Coord pNext = m_provider.getTargetPos(targetIdx, next);

  const Coord v = (pNext - pIdx) / m_targetTimeStep;
  const Coord curPos = interpolate(targetIdx, validTime);

  return curPos + v * dt;
}

}  // namespace BallisticApp

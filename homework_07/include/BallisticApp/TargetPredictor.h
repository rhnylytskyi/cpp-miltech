#pragma once

#include "BallisticApp/types/Coord.h"
#include "BallisticApp/interfaces/ITargetProvider.h"

namespace BallisticApp {

class TargetPredictor {
public:
  TargetPredictor(const ITargetProvider& provider, const float targetTimeStep);

  Coord interpolate(int targetIdx, float t) const;
  Coord extrapolate(int targetIdx, float time, float dt) const;

private:
  const ITargetProvider& m_provider;
  const float m_targetTimeStep;
};

}  // namespace BallisticApp

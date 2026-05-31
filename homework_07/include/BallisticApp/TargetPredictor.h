#pragma once

#include "BallisticApp/Coord.h"
#include "BallisticApp/DroneConfig.h"
#include "BallisticApp/interfaces/ITargetProvider.h"

namespace BallisticApp {

class TargetPredictor {
public:
  TargetPredictor(const ITargetProvider& provider, const DroneConfig& config);

  Coord interpolate(int targetIdx, float t) const;
  Coord extrapolate(int targetIdx, float time, float dt) const;

private:
  const ITargetProvider& m_provider;
  const DroneConfig& m_config;
};

}  // namespace BallisticApp

#pragma once

#include "ballistic_app/Types.h"
#include "ballistic_app/interfaces/ITargetProvider.h"

namespace BallisticApp {
class TargetPredictor {
public:
  TargetPredictor(ITargetProvider* provider, const DroneConfig& config);

  Coord interpolate(int targetIdx, float t) const;
  Coord extrapolate(int targetIdx, float time, float dt) const;

private:
  ITargetProvider* m_provider;
  DroneConfig m_config;
};
}  // namespace BallisticApp

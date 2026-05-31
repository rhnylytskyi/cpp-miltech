#pragma once

#include "BallisticApp/types/Coord.h"
#include <string>

namespace BallisticApp {

struct DroneConfig {
  std::string ammoName;
  Coord startPos{0.0f, 0.0f};

  float altitude{0.0f};
  float initialDir{0.0f};
  float attackSpeed{0.0f};
  float accelPath{0.0f};
  float arrayTimeStep{0.0f};
  float simTimeStep{0.01f};
  float hitRadius{0.0f};
  float angularSpeed{0.0f};
  float turnThreshold{0.0f};
};

}  // namespace BallisticApp

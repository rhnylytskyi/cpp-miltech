#pragma once

#include "BallisticApp/Coord.h"
#include <string>

namespace BallisticApp {

class DroneConfig {
public:
  Coord startPos;
  float altitude;
  float initialDir;
  float attackSpeed;
  float accelPath;
  std::string ammoName;
  float arrayTimeStep;
  float simTimeStep;
  float hitRadius;
  float angularSpeed;
  float turnThreshold;
};

}  // namespace BallisticApp
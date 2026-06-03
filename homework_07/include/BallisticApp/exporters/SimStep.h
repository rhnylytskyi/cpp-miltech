#pragma once

#include "BallisticApp/types/Coord.h"
#include "BallisticApp/types/DroneStateType.h"

namespace BallisticApp {

struct SimStep {
  Coord pos{0.0f, 0.0f};
  float direction{0.0f};
  DroneStateType state{DroneStateType::STOPPED};
  int targetIdx{-1};
  Coord dropPoint{0.0f, 0.0f};
  Coord aimPoint{0.0f, 0.0f};
  Coord predictedTarget{0.0f, 0.0f};
};

}  // namespace BallisticApp
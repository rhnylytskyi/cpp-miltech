#pragma once

#include "BallisticApp/types/Coord.h"
#include "BallisticApp/types/DroneStateType.h"

namespace BallisticApp {

struct SimStep {
  Coord pos;
  float direction;
  DroneStateType state;
  int targetIdx;
  Coord dropPoint;
  Coord aimPoint;
  Coord predictedTarget;
};

}  // namespace BallisticApp
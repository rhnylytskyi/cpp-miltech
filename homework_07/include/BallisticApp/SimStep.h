#pragma once

#include "BallisticApp/Coord.h"
#include "BallisticApp/DroneStateType.h"

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
#pragma once

#include "BallisticApp/Coord.h"
#include "BallisticApp/DroneState.h"

namespace BallisticApp {

struct SimStep {
  Coord pos;
  float direction;
  DroneState state;
  int targetIdx;
  Coord dropPoint;
  Coord aimPoint;
  Coord predictedTarget;
};

}  // namespace BallisticApp
#pragma once

#include "ballistic_app/dto/Coord.hpp"

namespace BallisticApp {
struct SimStep {
  Coord pos;
  float direction;
  int state;
  int targetIdx;
  Coord dropPoint;
  Coord aimPoint;
  Coord predictedTarget;
};
}  // namespace BallisticApp
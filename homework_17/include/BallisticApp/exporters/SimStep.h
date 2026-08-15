#pragma once

#include "BallisticApp/utils/MathUtils.h"
#include <string>

namespace BallisticApp {

struct SimStep {
  Coord pos{0.0f, 0.0f};
  float direction = 0.0f;
  std::string mode;
  int currentTarget = -1;
  Coord dropPoint{0.0f, 0.0f};
  Coord aimPoint{0.0f, 0.0f};
  Coord predictedTarget{0.0f, 0.0f};
  float timeSec = 0.0f;
};

}  // namespace BallisticApp

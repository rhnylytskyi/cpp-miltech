#pragma once
#include "BallisticApp/types/Coord.h"

namespace BallisticApp {

struct Target {
  Coord pos;       // поточна позиція цілі
  Coord velocity;  // поточна швидкість цілі

  Target()
    : pos{0.0f, 0.0f}
    , velocity{0.0f, 0.0f}
  {
  }

  Target(Coord p, Coord v)
    : pos(p)
    , velocity(v)
  {
  }
};

}  // namespace BallisticApp

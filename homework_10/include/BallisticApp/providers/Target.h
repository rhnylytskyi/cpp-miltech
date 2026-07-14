#pragma once
#include "BallisticApp/types/Coord.h"

namespace BallisticApp {

struct Target {
  Coord pos;       // current position of the target
  Coord velocity;  // current velocity of the target

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

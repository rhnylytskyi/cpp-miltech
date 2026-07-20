#pragma once
#include "BallisticApp/types/Coord.h"
#include "BallisticApp/types/DroneStateType.h"

namespace BallisticApp {

struct DroneTelemetry {
  Coord pos;
  float direction;
  float speed;
  DroneStateType state;
  float timeSecSinceStart;
};

}  // namespace BallisticApp

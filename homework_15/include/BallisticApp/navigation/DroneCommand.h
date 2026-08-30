#pragma once
#include "BallisticApp/types/DroneStateType.h"

namespace BallisticApp {

struct DroneCommand {
  DroneStateType stateType;
  float desiredDir;
};
}  // namespace BallisticApp

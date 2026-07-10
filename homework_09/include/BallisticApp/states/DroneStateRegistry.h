#pragma once

#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/types/DroneStateType.h"
#include <cmath>

namespace BallisticApp {

class DroneStateRegistry {
public:
  static IDroneState* getState(DroneStateType type);
};

}  // namespace BallisticApp

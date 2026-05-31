#pragma once

#include "BallisticApp/types/DroneStateType.h"
#include "BallisticApp/interfaces/IDroneState.h"

namespace BallisticApp {

class DroneStateRegistry {
public:
  static IDroneState* getState(DroneStateType type);
};

}  // namespace BallisticApp

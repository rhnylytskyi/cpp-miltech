#pragma once
#include "BallisticApp/interfaces/IDroneState.h"

namespace BallisticApp::states {

class DroneStateRegistry {
public:
  static IDroneState* getState(DroneStateType type);
};

}  // namespace BallisticApp::states

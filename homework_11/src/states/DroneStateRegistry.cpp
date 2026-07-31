#include "BallisticApp/states/DroneStateRegistry.h"
#include "BallisticApp/states/StateStopped.h"
#include "BallisticApp/states/StateTurning.h"
#include "BallisticApp/states/StateAccelerating.h"
#include "BallisticApp/states/StateMoving.h"
#include "BallisticApp/states/StateDecelerating.h"
#include "BallisticApp/DroneStateType.h"
#include <iostream>

namespace BallisticApp::states {

IDroneState* DroneStateRegistry::getState(DroneStateType type)
{
  static StateStopped stopped;
  static StateTurning turning;
  static StateAccelerating accelerating;
  static StateMoving moving;
  static StateDecelerating decelerating;

  switch (type) {
    case DroneStateType::STOPPED:
      return &stopped;
    case DroneStateType::TURNING:
      return &turning;
    case DroneStateType::ACCELERATING:
      return &accelerating;
    case DroneStateType::MOVING:
      return &moving;
    case DroneStateType::DECELERATING:
      return &decelerating;
    default:
      std::cerr << "[FSM Critical] Detected invalid or corrupted DroneStateType! Falling back to STOPPED.\n";
      return &stopped;
  }
}

}  // namespace BallisticApp::states

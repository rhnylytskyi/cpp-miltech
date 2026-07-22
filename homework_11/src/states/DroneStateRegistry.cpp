#include "BallisticApp/states/DroneStateRegistry.h"
#include "BallisticApp/states/Stopped.h"
#include "BallisticApp/states/Turning.h"
#include "BallisticApp/states/Accelerating.h"
#include "BallisticApp/states/Moving.h"
#include "BallisticApp/states/Decelerating.h"
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

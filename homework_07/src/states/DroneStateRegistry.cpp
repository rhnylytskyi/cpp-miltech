#include "BallisticApp/states/DroneStateRegistry.h"
#include "BallisticApp/states/StateStopped.h"
#include "BallisticApp/states/StateAccelerating.h"
#include "BallisticApp/states/StateDecelerating.h"
#include "BallisticApp/states/StateTurning.h"
#include "BallisticApp/states/StateMoving.h"
#include <unordered_map>
#include <memory>

namespace BallisticApp {

IDroneState* DroneStateRegistry::getState(DroneStateType type)
{
  static const std::unordered_map<DroneStateType, std::unique_ptr<IDroneState>> s_registry = []() {
    std::unordered_map<DroneStateType, std::unique_ptr<IDroneState>> pool;
    pool[DroneStateType::STOPPED] = std::make_unique<StateStopped>();
    pool[DroneStateType::ACCELERATING] = std::make_unique<StateAccelerating>();
    pool[DroneStateType::DECELERATING] = std::make_unique<StateDecelerating>();
    pool[DroneStateType::TURNING] = std::make_unique<StateTurning>();
    pool[DroneStateType::MOVING] = std::make_unique<StateMoving>();
    return pool;
  }();

  auto it = s_registry.find(type);
  return (it != s_registry.end()) ? it->second.get() : nullptr;
}

}  // namespace BallisticApp

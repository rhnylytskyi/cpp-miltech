#pragma once
#include "BallisticApp/config/DroneConfig.h"
#include "BallisticApp/states/DroneStateType.h"

namespace BallisticApp {

enum class DroneStateType;

class IDroneState {
public:
  virtual ~IDroneState() = default;

  virtual DroneStateType execute(float& speed, float& direction, float desiredDir, const DroneConfig& cfg) = 0;

  virtual DroneStateType getType() const = 0;
};

}  // namespace BallisticApp

#pragma once

#include "BallisticApp/DroneState.h"
#include <memory>

namespace BallisticApp {

struct DroneContext;

class IDroneState {
public:
  virtual ~IDroneState() = default;
  virtual std::unique_ptr<IDroneState> execute(DroneContext& ctx, float dt) = 0;
  virtual DroneState getType() const = 0;
};

}  // namespace BallisticApp

#pragma once

#include "ballistic_app/Types.h"

namespace BallisticApp {

class ITargetProvider {
public:
  virtual ~ITargetProvider() = default;

  virtual int getTargetCount() const = 0;
  virtual int getTimeSteps() const = 0;
  virtual Coord getTargetPos(int targetIdx, int timeIdx) const = 0;
};

}  // namespace BallisticApp

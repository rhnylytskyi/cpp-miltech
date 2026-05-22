#pragma once
#include "ballistic_app/dto/Coord.hpp"

namespace BallisticApp {
class ITargetProvider {
public:
  virtual int getTargetCount() = 0;
  virtual int getTimeSteps() = 0;
  virtual Coord getTargetPos(int targetIdx, int timeIdx) = 0;
  virtual ~ITargetProvider() {}
};
}  // namespace BallisticApp
#pragma once

#include <ballistic_app/interfaces/ITargetProvider.hpp>
#include <ballistic_app/dto/Coord.hpp>

namespace BallisticApp {

class JsonTargetProvider : public ITargetProvider {
private:
  int tgtCount;
  int timeSteps;
  Coord** targets;

public:
  JsonTargetProvider(const char* filepath);
  ~JsonTargetProvider() override;

  int getTargetCount() override;
  int getTimeSteps() override;
  Coord getTargetPos(int targetIdx, int timeIdx) override;
};

}  // namespace BallisticApp

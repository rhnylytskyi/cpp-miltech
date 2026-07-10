#pragma once

#include <ballistic_app/interfaces/ITargetProvider.h>
#include <ballistic_app/Types.h>

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

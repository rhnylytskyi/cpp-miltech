#pragma once

#include "BallisticApp/interfaces/ITargetProvider.h"
#include "BallisticApp/types/Coord.h"
#include <vector>
#include <string>

namespace BallisticApp {

class JsonTargetProvider : public ITargetProvider {
public:
  explicit JsonTargetProvider(const std::string& filepath);
  ~JsonTargetProvider() = default;

  int getTargetCount() const override;
  int getTimeSteps() const override;
  Coord getTargetPos(int targetIdx, int timeIdx) const override;

private:
  int tgtCount{0};
  int timeSteps{0};
  std::vector<std::vector<Coord>> targets;
};

}  // namespace BallisticApp

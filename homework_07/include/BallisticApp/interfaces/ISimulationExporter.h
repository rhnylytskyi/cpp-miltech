#pragma once

#include "BallisticApp/types/SimStep.h"
#include <vector>

namespace BallisticApp {

class ISimulationExporter {
public:
  virtual ~ISimulationExporter() = default;

  virtual void exportSimulation(const std::vector<SimStep>& history) const = 0;
};

}  // namespace BallisticApp

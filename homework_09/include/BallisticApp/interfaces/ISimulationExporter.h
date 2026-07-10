#pragma once

#include <vector>

namespace BallisticApp {

struct SimStep;

class ISimulationExporter {
public:
  virtual ~ISimulationExporter() = default;

  virtual void exportSimulation(const std::vector<SimStep>& history) const = 0;
};

}  // namespace BallisticApp

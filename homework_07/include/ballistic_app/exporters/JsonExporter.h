#pragma once

#include "ballistic_app/interfaces/ISimulationExporter.h"
#include "ballistic_app/Types.h"
#include <string>

namespace BallisticApp {

class JsonExporter : public ISimulationExporter {
public:
  explicit JsonExporter(std::string filepath);

  void exportSimulation(const SimStep* history, int totalSteps) const override;

private:
  std::string m_filePath;
};

}  // namespace BallisticApp

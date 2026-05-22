#pragma once

#include "ballistic_app/io/interfaces/ISimulationExporter.hpp"
#include "ballistic_app/dto/SimStep.hpp"
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

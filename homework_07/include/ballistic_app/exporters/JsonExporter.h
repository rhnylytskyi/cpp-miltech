#pragma once

#include "ballistic_app/interfaces/ISimulationExporter.h"
#include "ballistic_app/Types.h"
#include <string>
#include <vector>

namespace BallisticApp {

class JsonExporter : public ISimulationExporter {
public:
  explicit JsonExporter(const std::string& filePath);
  ~JsonExporter() override = default;

  void exportSimulation(const std::vector<SimStep>& history) const override;

private:
  std::string m_filePath;
};

}  // namespace BallisticApp

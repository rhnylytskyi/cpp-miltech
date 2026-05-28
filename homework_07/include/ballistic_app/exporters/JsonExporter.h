#pragma once

#include "ballistic_app/interfaces/ISimulationExporter.h"
#include "ballistic_app/Types.h"
#include <nlohmann/json_fwd.hpp>
#include <string>
#include <vector>

namespace BallisticApp {

void to_json(nlohmann::json& j, const Coord& c);
void to_json(nlohmann::json& j, const SimStep& step);

class JsonExporter : public ISimulationExporter {
public:
  explicit JsonExporter(const std::string& filePath);
  ~JsonExporter() override = default;

  void exportSimulation(const std::vector<SimStep>& history) const override;

private:
  std::string m_filePath;
};

}  // namespace BallisticApp

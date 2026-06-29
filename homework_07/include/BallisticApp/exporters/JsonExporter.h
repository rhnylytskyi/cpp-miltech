#pragma once

#include "BallisticApp/interfaces/ISimulationExporter.h"
#include <filesystem>
#include <vector>
#include <nlohmann/json_fwd.hpp>

namespace BallisticApp {

struct Coord;
struct SimStep;

void to_json(nlohmann::json& j, const Coord& c);
void to_json(nlohmann::json& j, const SimStep& step);

class JsonExporter : public ISimulationExporter {
public:
  explicit JsonExporter(std::filesystem::path filePath);
  ~JsonExporter() override = default;

  void exportSimulation(const std::vector<SimStep>& history) const override;

private:
  std::filesystem::path m_filePath;
};

}  // namespace BallisticApp

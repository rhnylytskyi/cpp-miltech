#include "BallisticApp/exporters/JsonExporter.h"
#include <fstream>
#include <stdexcept>
#include <format>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace BallisticApp {

void to_json(json& j, const Coord& c)
{
  j = json{{"x", c.x}, {"y", c.y}};
}

void to_json(json& j, const SimStep& step)
{
  j = json{{"position", step.pos},
           {"direction", step.direction},
           {"state", static_cast<int>(step.state)},
           {"targetIdx", step.targetIdx},
           {"dropPoint", step.dropPoint},
           {"aimPoint", step.aimPoint},
           {"predictedTarget", step.predictedTarget}};
}

JsonExporter::JsonExporter(const std::string& filePath)
  : m_filePath(filePath)
{
}

void JsonExporter::exportSimulation(const std::vector<SimStep>& history) const
{
  std::ofstream file(m_filePath);
  if (!file.is_open()) {
    throw std::runtime_error(std::format("JsonExporter Error: Could not open file for writing: \"{}\"", m_filePath));
  }

  json root;
  root["steps"] = history;
  root["totalSteps"] = history.size();

  file << root.dump(2);
  file.close();
}

}  // namespace BallisticApp

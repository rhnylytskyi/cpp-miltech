#include "BallisticApp/exporters/JsonExporter.h"
#include "BallisticApp/exporters/SimStep.h"
#include "BallisticApp/types/Coord.h"
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <format>
#include <string>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace BallisticApp {

void to_json(json& j, const Coord& c)
{
  j = json{{"x", c.x}, {"y", c.y}};
}

void to_json(json& j, const SimStep& step)
{
  std::string stateStr = std::format("{}", step.state);

  j = json{{"position", step.pos},
           {"direction", step.direction},
           {"state", stateStr},
           {"targetIndex", step.targetIdx},
           {"dropPoint", step.dropPoint},
           {"aimPoint", step.aimPoint},
           {"predictedTarget", step.predictedTarget},
           {"timeSecSinceStart", step.timeSecSinceStart}};
}

JsonExporter::JsonExporter(std::filesystem::path filePath)
  : m_filePath(std::move(filePath))
{
}

void JsonExporter::exportSimulation(const std::vector<SimStep>& history) const
{
  std::ofstream file(m_filePath);
  if (!file.is_open()) {
    throw std::runtime_error(std::format("JsonExporter Error: Could not open file for writing: \"{}\"", m_filePath.string()));
  }

  json root;
  root["steps"] = history;
  root["totalSteps"] = history.size();

  file << std::setw(2) << root;
}

}  // namespace BallisticApp

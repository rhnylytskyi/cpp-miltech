#include "ballistic_app/exporters/JsonExporter.h"
#include "ballistic_app/Defines.h"
#include <fstream>
#include <stdexcept>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace BallisticApp {

JsonExporter::JsonExporter(const std::string& filePath)
  : m_filePath(filePath)
{
}

void JsonExporter::exportSimulation(const std::vector<SimStep>& history) const
{
  if (history.empty()) {
    throw std::runtime_error("JsonExporter Error: No simulation steps provided!");
  }

  json jsonOut;
  jsonOut["totalSteps"] = history.size();
  jsonOut["steps"] = json::array();

  for (const auto& step : history) {
    json stepJson;

    stepJson["direction"] = step.direction;
    stepJson["state"] = step.state;
    stepJson["targetIdx"] = step.targetIdx;

    stepJson["pos"] = {{"x", step.pos.x}, {"y", step.pos.y}};
    stepJson["dropPoint"] = {{"x", step.dropPoint.x}, {"y", step.dropPoint.y}};
    stepJson["aimPoint"] = {{"x", step.aimPoint.x}, {"y", step.aimPoint.y}};
    stepJson["predictedTarget"] = {{"x", step.predictedTarget.x}, {"y", step.predictedTarget.y}};

    jsonOut["steps"].push_back(stepJson);
  }

  std::ofstream outFile(m_filePath);
  if (!outFile.is_open()) {
    throw std::runtime_error("JsonExporter Error: Could not open file for writing: " + m_filePath);
  }

  outFile << jsonOut.dump(2);
  outFile.close();
  LOG("Data successfully saved to " << m_filePath);
}

}  // namespace BallisticApp

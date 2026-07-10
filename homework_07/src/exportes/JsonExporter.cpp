#include "ballistic_app/exporters/JsonExporter.h"
#include "ballistic_app/Defines.h"
#include <fstream>
#include <stdexcept>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace BallisticApp {

JsonExporter::JsonExporter(std::string filePath)
  : m_filePath(filePath)
{
}

void JsonExporter::exportSimulation(const SimStep* history, int totalSteps) const
{
  if (!history || totalSteps <= 0) {
    throw std::runtime_error("JsonExporter Error: No simulation steps provided or totalSteps <= 0!");
  }

  json jsonOut;
  jsonOut["totalSteps"] = totalSteps;
  jsonOut["steps"] = json::array();

  for (int i = 0; i < totalSteps; ++i) {
    json stepJson;

    stepJson["direction"] = history[i].direction;
    stepJson["state"] = history[i].state;
    stepJson["targetIdx"] = history[i].targetIdx;

    stepJson["pos"] = {{"x", history[i].pos.x}, {"y", history[i].pos.y}};
    stepJson["dropPoint"] = {{"x", history[i].dropPoint.x}, {"y", history[i].dropPoint.y}};
    stepJson["aimPoint"] = {{"x", history[i].aimPoint.x}, {"y", history[i].aimPoint.y}};
    stepJson["predictedTarget"] = {{"x", history[i].predictedTarget.x}, {"y", history[i].predictedTarget.y}};

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

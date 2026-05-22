#include "ballistic_app/utils/JsonExporter.hpp"
#include "ballistic_app/Defines.hpp"
#include "ballistic_app/dto/SimStep.hpp"
#include <fstream>
#include <iostream> // IWYU pragma: keep
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace BallisticApp {

void saveSimulationToJson(const char* filepath, const SimStep* history, int totalSteps)
{
  if (!history || totalSteps <= 0) {
    LOG("Warning: No simulation steps to save!");
    return;
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

  std::ofstream outFile(filepath);
  if (outFile.is_open()) {
    outFile << jsonOut.dump(4);
    outFile.close();
    LOG("Data successfully saved to " << filepath);
  }
  else {
    LOG("Error: Could not open " << filepath << " for writing!");
  }
}

}  // namespace BallisticApp

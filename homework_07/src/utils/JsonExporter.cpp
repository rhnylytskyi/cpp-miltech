#include "ballistic_app/utils/JsonExporter.hpp"
#include "ballistic_app/Defines.hpp"
#include "ballistic_app/dto/SimStep.hpp"
#include <fstream>
#include <iostream>  // IWYU pragma: keep
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace BallisticApp {

void saveSimulationToJson(const char* filepath, const SimStep* history, int totalSteps)
{
  if (!history || totalSteps <= 0) {
    LOG("Warning: No simulation steps to save!");
    return;
  }

  json json_out;
  json_out["totalSteps"] = totalSteps;
  json_out["steps"] = json::array();

  for (int i = 0; i < totalSteps; ++i) {
    json step_json;

    step_json["direction"] = history[i].direction;
    step_json["state"] = history[i].state;
    step_json["targetIdx"] = history[i].targetIdx;

    step_json["pos"] = {{"x", history[i].pos.x}, {"y", history[i].pos.y}};
    step_json["dropPoint"] = {{"x", history[i].dropPoint.x}, {"y", history[i].dropPoint.y}};
    step_json["aimPoint"] = {{"x", history[i].aimPoint.x}, {"y", history[i].aimPoint.y}};
    step_json["predictedTarget"] = {{"x", history[i].predictedTarget.x}, {"y", history[i].predictedTarget.y}};

    json_out["steps"].push_back(step_json);
  }

  std::ofstream out_file(filepath);
  if (out_file.is_open()) {
    out_file << json_out.dump(4);
    out_file.close();
    LOG("Data successfully saved to " << filepath);
  }
  else {
    LOG("Error: Could not open " << filepath << " for writing!");
  }
}

}  // namespace BallisticApp

#include "BallisticApp/exporters/JsonExporter.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

namespace BallisticApp {

using json = nlohmann::json;

void JsonExporter::record(const SimStep& step)
{
  m_steps.push_back(step);
}

bool JsonExporter::save(const std::string& path) const
{
  json stepsArray = json::array();

  for (const auto& step : m_steps) {
    json jStep;

    jStep["position"]["x"] = step.pos.x;
    jStep["position"]["y"] = step.pos.y;
    jStep["direction"] = step.direction;
    jStep["state"] = step.mode;
    jStep["targetIndex"] = step.currentTarget;

    jStep["dropPoint"]["x"] = step.dropPoint.x;
    jStep["dropPoint"]["y"] = step.dropPoint.y;

    jStep["aimPoint"]["x"] = step.aimPoint.x;
    jStep["aimPoint"]["y"] = step.aimPoint.y;

    jStep["predictedTarget"]["x"] = step.predictedTarget.x;
    jStep["predictedTarget"]["y"] = step.predictedTarget.y;
    jStep["timeSecSinceStart"] = step.timeSec;

    stepsArray.push_back(jStep);
  }

  json root = json::object();
  root["steps"] = stepsArray;
  root["totalSteps"] = static_cast<int>(m_steps.size());

  std::ofstream file(path);
  if (!file.is_open()) {
    std::cerr << "[JsonExporter] Error: Failed to open output track file: " << path << "\n";
    return false;
  }

  file << root.dump(4);
  return true;
}

}  // namespace BallisticApp

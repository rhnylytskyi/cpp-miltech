#include "ballistic_app/providers/JsonTargetProvider.h"
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace BallisticApp {

JsonTargetProvider::JsonTargetProvider(const std::string& filepath)
{
  std::ifstream f(filepath);
  if (!f.is_open())
    return;

  json j_data;
  f >> j_data;
  f.close();

  tgtCount = j_data["targetCount"];
  timeSteps = j_data["timeSteps"];

  targets.reserve(tgtCount);

  for (const auto& jTarget : j_data["targets"]) {
    std::vector<Coord> target_positions;
    target_positions.reserve(timeSteps);

    for (const auto& j_pos : jTarget["positions"]) {
      Coord c;
      c.x = j_pos["x"];
      c.y = j_pos["y"];
      target_positions.push_back(c);
    }
    targets.push_back(std::move(target_positions));
  }
}

int JsonTargetProvider::getTargetCount() const
{
  return tgtCount;
}

int JsonTargetProvider::getTimeSteps() const
{
  return timeSteps;
}

Coord JsonTargetProvider::getTargetPos(int targetIdx, int timeIdx) const
{
  return targets.at(targetIdx).at(timeIdx);
}

}  // namespace BallisticApp

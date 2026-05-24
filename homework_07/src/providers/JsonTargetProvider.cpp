#include "ballistic_app/providers/JsonTargetProvider.h"
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace BallisticApp {

JsonTargetProvider::JsonTargetProvider(const char* filepath)
  : tgtCount(0)
  , timeSteps(0)
  , targets(nullptr)
{
  std::ifstream f(filepath);
  if (!f.is_open())
    return;

  json j_data;
  f >> j_data;
  f.close();

  tgtCount = j_data["targetCount"];
  timeSteps = j_data["timeSteps"];

  targets = new Coord*[tgtCount];
  for (int i = 0; i < tgtCount; i++) {
    targets[i] = new Coord[timeSteps];
    for (int k = 0; k < timeSteps; k++) {
      targets[i][k].x = j_data["targets"][i]["positions"][k]["x"];
      targets[i][k].y = j_data["targets"][i]["positions"][k]["y"];
    }
  }
}

int JsonTargetProvider::getTargetCount()
{
  return tgtCount;
}

int JsonTargetProvider::getTimeSteps()
{
  return timeSteps;
}

Coord JsonTargetProvider::getTargetPos(int targetIdx, int timeIdx)
{
  return targets[targetIdx][timeIdx];
}

JsonTargetProvider::~JsonTargetProvider()
{
  if (targets) {
    for (int i = 0; i < tgtCount; i++) {
      delete[] targets[i];
    }
    delete[] targets;
  }
}

}  // namespace BallisticApp

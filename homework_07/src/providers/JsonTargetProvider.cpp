#include "BallisticApp/providers/JsonTargetProvider.h"
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
  // Захист від невалідного індексу цілі
  if (targetIdx < 0 || targetIdx >= static_cast<int>(targets.size())) {
    return {0.0f, 0.0f};
  }

  const auto& target_path = targets[targetIdx];
  if (target_path.empty()) {
    return {0.0f, 0.0f};
  }

  // Захист від виходу за межі часових кроків (кадрів)
  if (timeIdx < 0) {
    return target_path.front();  // Повертаємо початкову позицію цілі
  }
  if (timeIdx >= static_cast<int>(target_path.size())) {
    return target_path.back();  // Повертаємо останню відому позицію цілі замість крашу!
  }

  // Якщо все супер — повертаємо швидкий доступ через оператор []
  return target_path[timeIdx];
}

}  // namespace BallisticApp

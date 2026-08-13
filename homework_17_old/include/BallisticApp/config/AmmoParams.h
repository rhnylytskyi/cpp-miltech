#pragma once

#include <string>

namespace BallisticApp {

struct AmmoParams {
  std::string name;
  float mass{0.0f};
  float drag{0.0f};
  float lift{0.0f};
};

}  // namespace BallisticApp
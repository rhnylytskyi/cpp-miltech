#pragma once
#include <string>

namespace BallisticApp {
struct AmmoParams {
  std::string name;
  float mass;
  float drag;
  float lift;
};
}  // namespace BallisticApp

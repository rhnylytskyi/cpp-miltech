#pragma once

#include <filesystem>

namespace BallisticApp {

struct AmmoParams;

class IBallisticSolver {
public:
  struct Result {
    float flightTime = 0.0f;
    float hDistance = 0.0f;
  };

  virtual ~IBallisticSolver() = default;

  virtual bool initialize(const std::filesystem::path& /*tablePath*/) { return true; }
  virtual Result calculate(float altitude, float speed, const AmmoParams& ammo) = 0;
};

}  // namespace BallisticApp

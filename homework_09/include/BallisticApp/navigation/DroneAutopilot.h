#pragma once

namespace BallisticApp {

struct MissionContext;

class DroneAutopilot {
public:
  DroneAutopilot() = default;

  /**
   * @brief Calculates one tick of the drone's autopilot navigation and integrates the physics of motion.
   * @param ctx The mission context (real or virtual for ballistic prediction).
   */
  void update(MissionContext& ctx) const;
};

}  // namespace BallisticApp

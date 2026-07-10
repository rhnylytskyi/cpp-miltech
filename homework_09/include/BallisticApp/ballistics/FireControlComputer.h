#pragma once

#include "BallisticApp/ballistics/FireSolution.h"

namespace BallisticApp {

class TargetExtrapolator;
class DroneAutopilot;
struct MissionContext;

class FireControlComputer {
public:
  FireControlComputer(TargetExtrapolator& extrapolator, const DroneAutopilot& autopilot, float simTimeStep);

  /**
   * @brief Calculates the ballistic solution for attacking a given target.
   *
   * @param currentMissionCtx The current real mission context of the drone.
   * @param targetIdx The index of the target for extrapolating its trajectory.
   * @return FireSolution The structure with the complete calculation result.
   */
  FireSolution calculateSolution(const MissionContext& currentMissionCtx, int targetIdx) const;

private:
  TargetExtrapolator& m_extrapolator;
  const DroneAutopilot& m_autopilot;
  float m_simTimeStep;
};

}  // namespace BallisticApp

#pragma once

#include "BallisticApp/types/Coord.h"

namespace BallisticApp {

/**
 * @brief Structure of the complex ballistic solution for attacking a target.
 */
struct FireSolution {
  int targetId{-1};
  bool isSuccess{false};              // Whether the mathematical solution was found
  float time{0.0f};                   // Flight time to the firing point/drop point
  Coord firePoint{0.0f, 0.0f};        // Optimal coordinate of the drop point
  Coord predictedTarget{0.0f, 0.0f};  // Predicted coordinates of the target at the moment of impact
};

}  // namespace BallisticApp

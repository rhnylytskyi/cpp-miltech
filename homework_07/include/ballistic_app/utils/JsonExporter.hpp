#pragma once

#include "ballistic_app/dto/SimStep.hpp"

namespace BallisticApp {

void saveSimulationToJson(const char* filepath, const SimStep* history, int totalSteps);

}  // namespace BallisticApp

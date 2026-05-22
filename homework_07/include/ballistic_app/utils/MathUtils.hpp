#pragma once

#include "ballistic_app/dto/Coord.hpp"

namespace BallisticApp::Math {

float length(Coord c);
Coord normalize(Coord c);
float normalizeAngle(float a);

}  // namespace BallisticApp::Math

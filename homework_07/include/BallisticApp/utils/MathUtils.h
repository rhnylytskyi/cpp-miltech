#pragma once

#include "BallisticApp/Coord.h"

namespace BallisticApp::Math {

float length(Coord c);
Coord normalize(Coord c);
float normalizeAngle(float a);

}  // namespace BallisticApp::Math

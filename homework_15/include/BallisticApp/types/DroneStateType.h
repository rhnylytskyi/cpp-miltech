#pragma once

#include <format>
#include <string_view>

namespace BallisticApp {

enum class DroneStateType : int { STOPPED = 0, ACCELERATING = 1, DECELERATING = 2, TURNING = 3, MOVING = 4 };

}

namespace std {

template <>
struct formatter<BallisticApp::DroneStateType> : formatter<std::string_view> {
  auto format(BallisticApp::DroneStateType state, std::format_context& ctx) const
  {
    std::string_view name = "UNKNOWN";
    switch (state) {
      case BallisticApp::DroneStateType::STOPPED:
        name = "STOPPED";
        break;
      case BallisticApp::DroneStateType::TURNING:
        name = "TURNING";
        break;
      case BallisticApp::DroneStateType::ACCELERATING:
        name = "ACCELERATING";
        break;
      case BallisticApp::DroneStateType::MOVING:
        name = "MOVING";
        break;
      case BallisticApp::DroneStateType::DECELERATING:
        name = "DECELERATING";
        break;
    }
    return formatter<std::string_view>::format(name, ctx);
  }
};

}  // namespace std

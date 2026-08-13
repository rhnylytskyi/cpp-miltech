#pragma once

#include <format>
#include <string_view>
#include <cstdint>

namespace BallisticApp {

enum class DroneStateType : uint8_t { STOPPED = 0, MOVING = 1, ACCELERATING = 2, DECELERATING = 3, TURNING = 4 };

}  // namespace BallisticApp

namespace std {

/**
 * @brief Custom std::format formatter specialization for DroneStateType logging support.
 */
template <>
struct formatter<BallisticApp::DroneStateType> : formatter<std::string_view> {
  auto format(BallisticApp::DroneStateType state, std::format_context& ctx) const
  {
    std::string_view name = "UNKNOWN";
    switch (state) {
      case BallisticApp::DroneStateType::STOPPED:
        name = "STOPPED";
        break;
      case BallisticApp::DroneStateType::MOVING:
        name = "MOVING";
        break;
      case BallisticApp::DroneStateType::ACCELERATING:
        name = "ACCELERATING";
        break;
      case BallisticApp::DroneStateType::DECELERATING:
        name = "DECELERATING";
        break;
      case BallisticApp::DroneStateType::TURNING:
        name = "TURNING";
        break;
    }
    return formatter<std::string_view>::format(name, ctx);
  }
};

}  // namespace std

#pragma once

#include <format>
#include <string_view>

namespace BallisticApp {

enum class DroneStateType {
  STOPPED = 0,       // Дрон стоїть на місці (початковий стан або після гальмування)
  TURNING = 1,       // Дрон розвертається на місці у напрямку точки скидання
  ACCELERATING = 2,  // Дрон розганяється до своєї максимальної швидкості
  MOVING = 3,        // Дрон летить на максимальній робочій швидкості (атака)
  DECELERATING = 4   // Дрон гальмує (якщо потрібно змінити курс або зупинитися)
};

}  // namespace BallisticApp

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

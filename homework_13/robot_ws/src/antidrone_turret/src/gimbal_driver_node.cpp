#include "antidrone_turret/msg/gimbal_command.hpp"
#include <cstdint>
#include <rclcpp/rclcpp.hpp>

namespace {
constexpr auto gimbalCmdTopic = "/gimbal/cmd";

const char *getDirectionName(const std::int8_t activeDirection)
{
  using GimbalCommand = antidrone_turret::msg::GimbalCommand;
  if (activeDirection == GimbalCommand::UP) {
    return "UP";
  }
  if (activeDirection == GimbalCommand::DOWN) {
    return "DOWN";
  }
  return "CENTER";
}
}  // namespace

// Gimbal vertical guidance driver node. Logs incoming command diagnostics.
class GimbalDriverNode final : public rclcpp::Node {
public:
  using GimbalCommand = antidrone_turret::msg::GimbalCommand;

  GimbalDriverNode()
    : Node("gimbal_driver_node")
  {
    gimbalSubscription = create_subscription<GimbalCommand>(
      gimbalCmdTopic, 10, [this](const GimbalCommand &incomingCommand) { onCommandReceived(incomingCommand); });

    RCLCPP_INFO(get_logger(), "listening on %s", gimbalCmdTopic);
  }

private:
  void onCommandReceived(const GimbalCommand &incomingCommand)
  {
    RCLCPP_INFO(get_logger(),
                "received: direction=%s target_y=%.1f error_y=%.1f",
                getDirectionName(incomingCommand.direction),
                incomingCommand.target_y,
                incomingCommand.error_y);
  }

  rclcpp::Subscription<GimbalCommand>::SharedPtr gimbalSubscription;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalDriverNode>());
  rclcpp::shutdown();
  return 0;
}

#include "antidrone_turret/msg/servo_command.hpp"
#include <cstdint>
#include <rclcpp/rclcpp.hpp>

namespace {
constexpr auto servoCmdTopic = "/servo/cmd";

const char *getDirectionName(const std::int8_t activeDirection)
{
  using ServoCommand = antidrone_turret::msg::ServoCommand;
  if (activeDirection == ServoCommand::RIGHT) {
    return "RIGHT";
  }
  if (activeDirection == ServoCommand::LEFT) {
    return "LEFT";
  }
  return "CENTER";
}
}  // namespace

// Yaw servo horizontal rotation driver node. Logs incoming command diagnostics.
class YawServoDriverNode final : public rclcpp::Node {
public:
  using ServoCommand = antidrone_turret::msg::ServoCommand;

  YawServoDriverNode()
    : Node("yaw_servo_driver_node")
  {
    servoSubscription = create_subscription<ServoCommand>(
      servoCmdTopic, 10, [this](const ServoCommand &incomingCommand) { onCommandReceived(incomingCommand); });

    RCLCPP_INFO(get_logger(), "listening on %s", servoCmdTopic);
  }

private:
  void onCommandReceived(const ServoCommand &incomingCommand)
  {
    RCLCPP_INFO(get_logger(),
                "received: direction=%s target_x=%.1f error_x=%.1f",
                getDirectionName(incomingCommand.direction),
                incomingCommand.target_x,
                incomingCommand.error_x);
  }

  rclcpp::Subscription<ServoCommand>::SharedPtr servoSubscription;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YawServoDriverNode>());
  rclcpp::shutdown();
  return 0;
}

#include "antidrone_turret/msg/actuator_status.hpp"
#include "antidrone_turret/msg/gimbal_command.hpp"
#include "antidrone_turret/msg/servo_command.hpp"
#include "antidrone_turret/msg/target.hpp"
#include "antidrone_turret/msg/turret_status.hpp"
#include "antidrone_turret/srv/trigger_actuator.hpp"
#include "antidrone_turret/turret_decision.hpp"
#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace {
constexpr auto targetTopic = "/perception/target";
constexpr auto actuatorStatusTopic = "/actuator/status";
constexpr auto gimbalCmdTopic = "/gimbal/cmd";
constexpr auto servoCmdTopic = "/servo/cmd";
constexpr auto turretStatusTopic = "/turret/status";
constexpr auto triggerService = "/actuator/trigger";

antidrone_turret::msg::GimbalCommand convertToGimbalMessage(const antidrone_turret::GimbalCommandDecision &gimbalDecision)
{
  auto outMessage = antidrone_turret::msg::GimbalCommand{};
  outMessage.direction = static_cast<std::int8_t>(gimbalDecision.direction);
  outMessage.target_y = gimbalDecision.target_y;
  outMessage.error_y = gimbalDecision.error_y;
  return outMessage;
}

antidrone_turret::msg::ServoCommand convertToServoMessage(const antidrone_turret::ServoCommandDecision &servoDecision)
{
  auto outMessage = antidrone_turret::msg::ServoCommand{};
  outMessage.direction = static_cast<std::int8_t>(servoDecision.direction);
  outMessage.target_x = servoDecision.target_x;
  outMessage.error_x = servoDecision.error_x;
  return outMessage;
}

antidrone_turret::msg::TurretStatus convertToStatusMessage(const antidrone_turret::TurretDecision &turretDecision)
{
  auto outMessage = antidrone_turret::msg::TurretStatus{};
  outMessage.target_state = static_cast<std::uint8_t>(turretDecision.target_state);
  outMessage.action = static_cast<std::uint8_t>(turretDecision.action);
  outMessage.trigger_state = static_cast<std::uint8_t>(turretDecision.trigger_state);
  outMessage.confidence = turretDecision.confidence;
  outMessage.distance_m = turretDecision.distance_m;
  return outMessage;
}
}  // namespace

class TurretControllerNode final : public rclcpp::Node {
public:
  using Target = antidrone_turret::msg::Target;
  using ActuatorStatus = antidrone_turret::msg::ActuatorStatus;
  using GimbalCommand = antidrone_turret::msg::GimbalCommand;
  using ServoCommand = antidrone_turret::msg::ServoCommand;
  using TurretStatus = antidrone_turret::msg::TurretStatus;
  using TriggerActuator = antidrone_turret::srv::TriggerActuator;

  TurretControllerNode()
    : Node("turret_controller_node")
  {
    nodeConfig.confidence_threshold = static_cast<float>(declare_parameter<double>("confidence_threshold", 0.80));
    nodeConfig.max_distance_m = static_cast<float>(declare_parameter<double>("max_distance_m", 30.0));

    gimbalPublisher = create_publisher<GimbalCommand>(gimbalCmdTopic, 10);
    servoPublisher = create_publisher<ServoCommand>(servoCmdTopic, 10);
    statusPublisher = create_publisher<TurretStatus>(turretStatusTopic, 10);

    triggerClient = create_client<TriggerActuator>(triggerService);

    actuatorStatusSubscription = create_subscription<ActuatorStatus>(
      actuatorStatusTopic, 10, [this](const ActuatorStatus &incomingStatus) { onActuatorStatusReceived(incomingStatus); });

    targetSubscription =
      create_subscription<Target>(targetTopic, 10, [this](const Target &incomingTarget) { onTargetReceived(incomingTarget); });

    RCLCPP_INFO(get_logger(), "confidence_threshold=%.2f max_distance_m=%.1f", nodeConfig.confidence_threshold, nodeConfig.max_distance_m);
  }

private:
  void onActuatorStatusReceived(const ActuatorStatus &incomingStatus) { isActuatorReady = (incomingStatus.state == ActuatorStatus::READY); }

  void onTargetReceived(const Target &incomingTarget)
  {
    antidrone_turret::TurretDecisionInput evaluationInput;
    evaluationInput.visible = incomingTarget.visible;
    evaluationInput.x = incomingTarget.x;
    evaluationInput.y = incomingTarget.y;
    evaluationInput.distance_m = incomingTarget.distance_m;
    evaluationInput.confidence = incomingTarget.confidence;
    evaluationInput.actuator_ready = isActuatorReady;

    const auto calculatedDecision = antidrone_turret::evaluate(evaluationInput, nodeConfig);

    if (calculatedDecision.has_gimbal_command) {
      gimbalPublisher->publish(convertToGimbalMessage(calculatedDecision.gimbal_command));
    }

    if (calculatedDecision.has_servo_command) {
      servoPublisher->publish(convertToServoMessage(calculatedDecision.servo_command));
    }

    statusPublisher->publish(convertToStatusMessage(calculatedDecision));

    if (calculatedDecision.trigger_state == antidrone_turret::TriggerDecision::kRequested) {
      executeTriggerRequest(incomingTarget.confidence, incomingTarget.distance_m);
    }
  }

  void executeTriggerRequest(const float targetConfidence, const float targetDistance)
  {
    if (!triggerClient->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "trigger service not ready, skipping request");
      return;
    }

    auto serviceRequest = std::make_shared<TriggerActuator::Request>();
    serviceRequest->confidence = targetConfidence;
    serviceRequest->distance_m = targetDistance;

    triggerClient->async_send_request(serviceRequest, [this](rclcpp::Client<TriggerActuator>::SharedFuture responseFuture) {
      const auto serviceResponse = responseFuture.get();
      RCLCPP_INFO(get_logger(),
                  "trigger response accepted=%s trigger_count=%u",
                  serviceResponse->accepted ? "true" : "false",
                  serviceResponse->trigger_count);
    });
  }

  antidrone_turret::TurretDecisionConfig nodeConfig{};

  // Fail-safe: assume actuator is NOT ready until the first status message arrives.
  // ROS 2 message ordering is not guaranteed during startup sequences.
  bool isActuatorReady{false};

  rclcpp::Subscription<Target>::SharedPtr targetSubscription;
  rclcpp::Subscription<ActuatorStatus>::SharedPtr actuatorStatusSubscription;
  rclcpp::Publisher<GimbalCommand>::SharedPtr gimbalPublisher;
  rclcpp::Publisher<ServoCommand>::SharedPtr servoPublisher;
  rclcpp::Publisher<TurretStatus>::SharedPtr statusPublisher;
  rclcpp::Client<TriggerActuator>::SharedPtr triggerClient;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurretControllerNode>());
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp/rclcpp.hpp"
#include "underground_world/msg/enemy_down.hpp"
#include "underground_world/srv/payload_trigger.hpp"

namespace {

constexpr auto kTargetDownTopic = "/payload/enemy_down";
constexpr auto kActionService = "/payload/trigger";

using underground_world::msg::EnemyDown;
using underground_world::srv::PayloadTrigger;

class WeaponActionNode final : public rclcpp::Node {
public:
  WeaponActionNode()
    : Node("weapon_action_node")
  {
    m_targetDownPub = create_publisher<EnemyDown>(kTargetDownTopic, rclcpp::QoS{10});

    m_actionService = create_service<PayloadTrigger>(
      kActionService,
      [this](const std::shared_ptr<rmw_request_id_t>,
             const std::shared_ptr<PayloadTrigger::Request> request,
             std::shared_ptr<PayloadTrigger::Response> response) {
        EnemyDown reportMsg;
        reportMsg.contact_id = request->contact_id;
        reportMsg.x = request->x;
        reportMsg.y = request->y;

        m_targetDownPub->publish(reportMsg);

        response->accepted = true;
        response->reason = "Target eliminated successfully";

        RCLCPP_INFO(get_logger(), "Weapon triggered: contact_id=%d at coordinates (%d, %d)", request->contact_id, request->x, request->y);
      });
  }

private:
  rclcpp::Publisher<EnemyDown>::SharedPtr m_targetDownPub;
  rclcpp::Service<PayloadTrigger>::SharedPtr m_actionService;
};

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WeaponActionNode>());
  rclcpp::shutdown();
  return 0;
}

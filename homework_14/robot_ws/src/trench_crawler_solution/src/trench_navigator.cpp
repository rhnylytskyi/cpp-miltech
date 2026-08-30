#include "rclcpp/rclcpp.hpp"
#include "underground_world/msg/local_scan.hpp"
#include "underground_world/msg/move_command.hpp"
#include "underground_world/msg/robot_result.hpp"
#include "underground_world/msg/student_status.hpp"
#include "underground_world/scenario.hpp"
#include "underground_world/state_qos.hpp"
#include "underground_world/srv/payload_trigger.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <map>
#include <optional>
#include <queue>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace {

using underground_world::Position;
using underground_world::msg::LocalScan;
using underground_world::msg::MoveCommand;
using underground_world::msg::RobotResult;
using underground_world::msg::StudentStatus;
using underground_world::srv::PayloadTrigger;

constexpr auto kScanTopic = "/robot/local_scan";
constexpr auto kMoveTopic = "/robot/cmd_move";
constexpr auto kResultTopic = "/robot/result";
constexpr auto kStatusTopic = "/student/status";
constexpr auto kTriggerService = "/payload/trigger";

constexpr char kWall = '#';
constexpr char kUnknown = '?';

// Determine movement command from positions
std::uint8_t getDirectionFromStep(const Position& start, const Position& target)
{
  const Position offset{target.x - start.x, target.y - start.y};
  if (offset.x == 0 && offset.y == -1)
    return MoveCommand::UP;
  if (offset.x == 0 && offset.y == 1)
    return MoveCommand::DOWN;
  if (offset.x == -1 && offset.y == 0)
    return MoveCommand::LEFT;
  return MoveCommand::RIGHT;
}

struct EnemyContact {
  int contactId = 0;
  Position pos;
};
class TrenchNavigatorNode final : public rclcpp::Node {
public:
  TrenchNavigatorNode()
    : Node("trench_navigator_node")
  {
    // ROS2 subscribers with required QoS
    m_scanSub = create_subscription<LocalScan>(
      kScanTopic, underground_world::make_state_qos(), [this](const LocalScan::SharedPtr msg) { handleLocalScan(*msg); });

    m_resultSub = create_subscription<RobotResult>(
      kResultTopic, underground_world::make_state_qos(), [this](const RobotResult::SharedPtr msg) { m_latestResult = *msg; });

    // ROS2 publishers and service client
    m_movePub = create_publisher<MoveCommand>(kMoveTopic, rclcpp::QoS{10});
    m_statusPub = create_publisher<StudentStatus>(kStatusTopic, rclcpp::QoS{10});
    m_triggerClient = create_client<PayloadTrigger>(kTriggerService);

    // Main execution loop (20ms)
    m_loopTimer = create_wall_timer(std::chrono::milliseconds{20}, [this]() { processControlTick(); });
  }

private:
  void handleLocalScan(const LocalScan& scan)
  {
    m_hasScanData = true;
    m_isNewScanAvailable = true;
    m_currentPos = Position{scan.robot_x, scan.robot_y};
    m_visibleEnemies.clear();

    // Update internal map memory
    for (const auto& cell : scan.cells) {
      const Position p{cell.x, cell.y};
      m_gridMap[p] = cell.cell_type.empty() ? kUnknown : cell.cell_type.front();

      if (cell.cell_type == "C") {
        m_activeEnemies[cell.contact_id] = p;
        m_visibleEnemies.push_back(EnemyContact{cell.contact_id, p});
      }
      else if (cell.cell_type == "x") {
        m_activeEnemies.erase(cell.contact_id);
        m_neutralizedEnemies.insert(cell.contact_id);
      }
    }

    // Clean up obsolete targets
    for (auto it = m_activeEnemies.begin(); it != m_activeEnemies.end();) {
      if (m_neutralizedEnemies.contains(it->first)) {
        it = m_activeEnemies.erase(it);
      }
      else {
        ++it;
      }
    }

    // Check if currently targeted enemy is still visible
    if (m_currentTargetId.has_value()) {
      bool isStillVisible = std::any_of(m_visibleEnemies.begin(), m_visibleEnemies.end(), [this](const EnemyContact& enemy) {
        return enemy.contactId == m_currentTargetId.value();
      });
      if (!isStillVisible) {
        m_currentTargetId.reset();
      }
    }
  }
  void processControlTick()
  {
    // Prevent race conditions using scan availability flag
    if (!m_hasScanData || !m_isNewScanAvailable) {
      return;
    }

    bool commandExecuted = false;

    if (m_latestResult.mission_result == "SUCCESS") {
      broadcastStatus(StudentStatus::DONE);
      commandExecuted = true;
    }
    else if (m_latestResult.mission_result == "FAILED_MAX_STEPS") {
      broadcastStatus(StudentStatus::FAILED);
      commandExecuted = true;
    }
    else if (m_currentTargetId.has_value()) {
      broadcastStatus(StudentStatus::ENGAGING);
      commandExecuted = true;
    }
    else if (!m_visibleEnemies.empty()) {
      commandExecuted = attackEnemyContact(m_visibleEnemies.front());
    }
    else {
      const auto nextStep = calculateNextStep();
      if (!nextStep.has_value()) {
        broadcastStatus(StudentStatus::DONE);
        commandExecuted = true;
      }
      else {
        MoveCommand cmd;
        cmd.direction = getDirectionFromStep(m_currentPos, nextStep.value());
        m_movePub->publish(cmd);
        broadcastStatus(StudentStatus::EXPLORING);
        commandExecuted = true;
      }
    }

    if (commandExecuted) {
      m_isNewScanAvailable = false;
    }
  }

  bool attackEnemyContact(const EnemyContact& enemy)
  {
    broadcastStatus(StudentStatus::ENGAGING);
    if (!m_triggerClient->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for payload trigger...");
      return false;
    }

    auto req = std::make_shared<PayloadTrigger::Request>();
    req->contact_id = enemy.contactId;
    req->x = enemy.pos.x;
    req->y = enemy.pos.y;

    m_currentTargetId = enemy.contactId;

    m_triggerClient->async_send_request(req, [this](rclcpp::Client<PayloadTrigger>::SharedFuture future) {
      const auto res = future.get();
      if (!res->accepted) {
        RCLCPP_WARN(get_logger(), "Weapon system rejected request: %s", res->reason.c_str());
        m_currentTargetId.reset();
      }
    });
    return true;
  }

  std::optional<Position> calculateNextStep() const
  {
    std::queue<Position> queue;
    std::map<Position, Position> traceMap;
    std::set<Position> visited;

    queue.push(m_currentPos);
    visited.insert(m_currentPos);

    // BFS traversal for mapping frontiers
    while (!queue.empty()) {
      const Position current = queue.front();
      queue.pop();

      if (checkIfFrontier(current)) {
        return extractFirstStep(current, traceMap);
      }

      for (const auto& next : getCardinalNeighbors(current)) {
        if (visited.contains(next) || !checkIfWalkable(next)) {
          continue;
        }
        visited.insert(next);
        traceMap.emplace(next, current);
        queue.push(next);
      }
    }
    return std::nullopt;
  }

  std::optional<Position> extractFirstStep(const Position target, const std::map<Position, Position>& traceMap) const
  {
    if (target == m_currentPos) {
      for (const auto& next : getCardinalNeighbors(m_currentPos)) {
        if (checkIfWalkable(next) && checkIfFrontier(next)) {
          return next;
        }
      }
      return std::nullopt;
    }

    Position step = target;
    auto it = traceMap.find(step);
    while (it != traceMap.end() && !(it->second == m_currentPos)) {
      step = it->second;
      it = traceMap.find(step);
    }

    if (it == traceMap.end()) {
      return std::nullopt;
    }
    return step;
  }

  bool checkIfFrontier(const Position& pos) const
  {
    if (!checkIfWalkable(pos)) {
      return false;
    }
    for (const auto& next : getCardinalNeighbors(pos)) {
      if (!m_gridMap.contains(next)) {
        return true;
      }
    }
    return false;
  }

  bool checkIfWalkable(const Position& pos) const
  {
    const auto it = m_gridMap.find(pos);
    if (it == m_gridMap.end() || it->second == kWall || it->second == kUnknown) {
      return false;
    }
    for (const auto& [id, enemyPos] : m_activeEnemies) {
      if (!m_neutralizedEnemies.contains(id) && enemyPos == pos) {
        return false;
      }
    }
    return true;
  }

  std::vector<Position> getCardinalNeighbors(const Position& pos) const
  {
    return {Position{pos.x, pos.y - 1}, Position{pos.x, pos.y + 1}, Position{pos.x - 1, pos.y}, Position{pos.x + 1, pos.y}};
  }

  void broadcastStatus(std::uint8_t state) const
  {
    StudentStatus msg;
    msg.state = state;
    m_statusPub->publish(msg);
  }

  // Class members with m_ prefix
  bool m_hasScanData = false;
  bool m_isNewScanAvailable = false;
  Position m_currentPos{};
  std::optional<int> m_currentTargetId;
  RobotResult m_latestResult{};

  std::map<Position, char> m_gridMap;
  std::map<int, Position> m_activeEnemies;
  std::set<int> m_neutralizedEnemies;
  std::vector<EnemyContact> m_visibleEnemies;

  rclcpp::Subscription<LocalScan>::SharedPtr m_scanSub;
  rclcpp::Subscription<RobotResult>::SharedPtr m_resultSub;
  rclcpp::Publisher<MoveCommand>::SharedPtr m_movePub;
  rclcpp::Publisher<StudentStatus>::SharedPtr m_statusPub;
  rclcpp::Client<PayloadTrigger>::SharedPtr m_triggerClient;
  rclcpp::TimerBase::SharedPtr m_loopTimer;
};

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrenchNavigatorNode>());
  rclcpp::shutdown();
  return 0;
}

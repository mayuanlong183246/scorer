// Copyright 2026 RMUC contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "behaviortree_cpp/action_node.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "pb2025_sentry_behavior/tactical_navigation.hpp"

namespace pb2025_sentry_behavior
{
class TacticalNavigateToPoseAction : public BT::StatefulActionNode
{
public:
  TacticalNavigateToPoseAction(const std::string & name, const BT::NodeConfig & config,
                               const BT::RosNodeParams & params)
  : BT::StatefulActionNode(name, config), node_(params.nh.lock()) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("goal"),
      BT::InputPort<std::string>("action_name", "navigate_to_pose", "Nav2 action name"),
      BT::InputPort<int>("timeout_ms", 60000, "Per-leg navigation timeout")};
  }
  BT::NodeStatus onStart() override
  {
    navigation_ = config().blackboard->get<std::shared_ptr<TacticalNavigation>>(
      "@tactical_navigation");
    started_ = node_->now().nanoseconds();
    auto goal = getInput<std::string>("goal");
    if (!goal) { return fail("missing_goal"); }
    bool validate = false;
    (void)config().blackboard->get("@validate_navigation_goals", validate);
    if (validate && !validMapGoal(*goal)) { return fail("goal_outside_free_map"); }
    if (!navigation_->start(*goal, getInput<std::string>("action_name").value())) {
      return fail(navigation_->busy() ? "navigation_handoff_pending" : navigation_->reason());
    }
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override
  {
    if (!navigation_->busy()) {
      return navigation_->succeeded() ? BT::NodeStatus::SUCCESS : fail(navigation_->reason());
    }
    if (node_->now().nanoseconds() - started_ >=
        int64_t(getInput<int>("timeout_ms").value()) * 1000000) {
      navigation_->cancel();
      return fail("navigation_timeout");
    }
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override { if (navigation_) { navigation_->cancel(); } }
private:
  bool validMapGoal(const std::string & text)
  {
    nav_msgs::msg::OccupancyGrid map;
    if (!config().blackboard->get("@tactical_map", map) ||
        map.header.frame_id != "map" || map.info.resolution <= 0) { return false; }
    try {
      const auto parts = BT::splitString(text, ';');
      if (parts.size() != 3 && parts.size() != 7) { return false; }
      const auto & origin = map.info.origin;
      const double dx = BT::convertFromString<double>(parts[0]) - origin.position.x;
      const double dy = BT::convertFromString<double>(parts[1]) - origin.position.y;
      const auto & q = origin.orientation;
      const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y * q.y + q.z * q.z));
      const double x = std::floor((std::cos(yaw) * dx + std::sin(yaw) * dy) / map.info.resolution);
      const double y = std::floor((-std::sin(yaw) * dx + std::cos(yaw) * dy) / map.info.resolution);
      if (!std::isfinite(x) || !std::isfinite(y) || x < 0 || y < 0 ||
          x >= map.info.width || y >= map.info.height) { return false; }
      const auto index = static_cast<size_t>(y) * map.info.width + static_cast<size_t>(x);
      return index < map.data.size() && map.data[index] >= 0 && map.data[index] < 50;
    } catch (const std::exception &) { return false; }
  }
  BT::NodeStatus fail(const std::string & reason)
  {
    config().blackboard->set("@tactical_failure_reason", reason);
    return BT::NodeStatus::FAILURE;
  }
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<TacticalNavigation> navigation_;
  int64_t started_{0};
};
}  // namespace pb2025_sentry_behavior
CreateRosNodePlugin(pb2025_sentry_behavior::TacticalNavigateToPoseAction, "TacticalNavigateToPose");

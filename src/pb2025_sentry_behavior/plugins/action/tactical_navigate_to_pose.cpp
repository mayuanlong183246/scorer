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
      BT::InputPort<int>("timeout_ms", 60000, "Per-leg navigation timeout"),
      BT::InputPort<int>("server_wait_timeout_ms", 10000, "Nav2 server startup timeout")};
  }
  BT::NodeStatus onStart() override
  {
    navigation_ = config().blackboard->get<std::shared_ptr<TacticalNavigation>>(
      "@tactical_navigation");
    wait_started_ = std::chrono::steady_clock::now();
    navigation_started_ = false;
    waiting_reported_ = false;
    const auto timeout = getInput<int>("timeout_ms");
    if (!timeout || *timeout <= 0) { return fail("invalid_navigation_timeout"); }
    const auto wait_timeout = getInput<int>("server_wait_timeout_ms");
    if (!wait_timeout || *wait_timeout <= 0) { return fail("invalid_server_wait_timeout"); }
    const auto goal = getInput<std::string>("goal");
    if (!goal) { return fail("missing_goal"); }
    goal_ = *goal;
    action_name_ = getInput<std::string>("action_name").value();
    bool validate = false;
    (void)config().blackboard->get("@validate_navigation_goals", validate);
    if (validate && !validMapGoal(goal_)) { return fail("goal_outside_free_map"); }
    return onRunning();
  }
  BT::NodeStatus onRunning() override
  {
    if (!navigation_started_) {
      if (!navigation_->actionServerReady(action_name_)) {
        if (navigation_->reason() == "invalid_action_name") {
          return fail("invalid_action_name");
        }
        if (!waiting_reported_) {
          navigation_->reportWaitingForServer(action_name_);
          waiting_reported_ = true;
        }
        if (std::chrono::steady_clock::now() - wait_started_ >=
            std::chrono::milliseconds(getInput<int>("server_wait_timeout_ms").value())) {
          return fail("navigation_server_unavailable");
        }
        return BT::NodeStatus::RUNNING;
      }
      if (!navigation_->start(goal_, action_name_)) {
        return fail(navigation_->busy() ? "navigation_handoff_pending" : navigation_->reason());
      }
      navigation_started_ = true;
      navigation_started_at_ = std::chrono::steady_clock::now();
      return BT::NodeStatus::RUNNING;
    }
    if (!navigation_->busy()) {
      return navigation_->succeeded() ? BT::NodeStatus::SUCCESS : fail(navigation_->reason());
    }
    if (std::chrono::steady_clock::now() - navigation_started_at_ >=
        std::chrono::milliseconds(getInput<int>("timeout_ms").value())) {
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
  std::string goal_, action_name_;
  std::chrono::steady_clock::time_point wait_started_, navigation_started_at_;
  bool navigation_started_{false};
  bool waiting_reported_{false};
};
}  // namespace pb2025_sentry_behavior
CreateRosNodePlugin(pb2025_sentry_behavior::TacticalNavigateToPoseAction, "TacticalNavigateToPose");

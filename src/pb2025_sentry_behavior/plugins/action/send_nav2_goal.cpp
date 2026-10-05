// Copyright 2025 Lihan Chen
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

#include "pb2025_sentry_behavior/plugins/action/send_nav2_goal.hpp"

#include <cmath>

#include "behaviortree_cpp/basic_types.h"

namespace pb2025_sentry_behavior
{

namespace
{
constexpr const char * kLightBlue = "\033[1;94m";
constexpr const char * kColorReset = "\033[0m";

std::string blueLog(const std::string & message)
{
  return std::string(kLightBlue) + message + kColorReset;
}

geometry_msgs::msg::Pose parseGoalString(const std::string & goal_string)
{
  auto parts = BT::splitString(goal_string, ';');
  geometry_msgs::msg::Pose pose;

  if (parts.size() == 3) {
    const double x = BT::convertFromString<double>(parts[0]);
    const double y = BT::convertFromString<double>(parts[1]);
    const double yaw = BT::convertFromString<double>(parts[2]);
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(yaw * 0.5);
    pose.orientation.w = std::cos(yaw * 0.5);
    return pose;
  }

  if (parts.size() == 7) {
    pose.position.x = BT::convertFromString<double>(parts[0]);
    pose.position.y = BT::convertFromString<double>(parts[1]);
    pose.position.z = BT::convertFromString<double>(parts[2]);
    pose.orientation.x = BT::convertFromString<double>(parts[3]);
    pose.orientation.y = BT::convertFromString<double>(parts[4]);
    pose.orientation.z = BT::convertFromString<double>(parts[5]);
    pose.orientation.w = BT::convertFromString<double>(parts[6]);
    return pose;
  }

  throw BT::RuntimeError("Invalid goal format. Expected x;y;yaw or x;y;z;qx;qy;qz;qw");
}
}  // namespace

SendNav2GoalAction::SendNav2GoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
{
  auto action_name = getInput<std::string>("action_name");
  if (action_name) {
    RCLCPP_INFO(logger(), "\033[1;92m[SendNav2Goal] configured action_name=%s\033[0m", action_name->c_str());
  } else {
    RCLCPP_WARN(logger(), "\033[1;92m[SendNav2Goal] action_name input missing, plugin will use default resolution\033[0m");
  }
}

rclcpp::Logger SendNav2GoalAction::logger()
{
  if (auto node = node_.lock()) {
    return node->get_logger();
  }
  return rclcpp::get_logger("SendNav2GoalAction");
}

bool SendNav2GoalAction::setGoal(nav2_msgs::action::NavigateToPose::Goal & goal)
{
  auto receive_goal = getInput<std::string>("goal");
  if (!receive_goal) {
    RCLCPP_ERROR(logger(), "Missing required input port: goal");
    return false;
  }

  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = now();
  goal.pose.pose = parseGoalString(*receive_goal);

  RCLCPP_INFO(
    logger(), "%s",
    blueLog(
      "[BT][SendNav2Goal] node=" + name() + " goal=" + *receive_goal + " frame=map")
      .c_str());

  return true;
}

BT::NodeStatus SendNav2GoalAction::onResultReceived(const WrappedResult & wr)
{
  switch (wr.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(logger(), "Navigation succeeded!");
      return BT::NodeStatus::SUCCESS;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(logger(), "Navigation aborted by server");
      return BT::NodeStatus::FAILURE;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(logger(), "Navigation canceled");
      return BT::NodeStatus::FAILURE;

    default:
      RCLCPP_ERROR(logger(), "Unknown navigation result code: %d", static_cast<int>(wr.code));
      return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus SendNav2GoalAction::onFeedback(
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  RCLCPP_DEBUG(logger(), "Distance remaining: %f", feedback->distance_remaining);
  return BT::NodeStatus::RUNNING;
}

void SendNav2GoalAction::onHalt() { RCLCPP_INFO(logger(), "SendNav2GoalAction has been halted."); }

BT::NodeStatus SendNav2GoalAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "SendNav2GoalAction failed with error code: %d", error);
  return BT::NodeStatus::FAILURE;
}

BT::PortsList SendNav2GoalAction::providedPorts()
{
  BT::PortsList additional_ports = {
    BT::InputPort<std::string>(
      "goal", "0;0;0", "Expected goal pose that send to nav2. Fill with format `x;y;yaw`"),
    BT::InputPort<std::string>("action_name", "navigate_to_pose", "Action server name"),
  };
  return providedBasicPorts(additional_ports);
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::SendNav2GoalAction, "SendNav2Goal");

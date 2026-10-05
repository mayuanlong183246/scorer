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

#include "pb2025_sentry_behavior/plugins/action/pub_nav2_goal.hpp"

#include <cmath>

#include "behaviortree_cpp/basic_types.h"

namespace pb2025_sentry_behavior
{

namespace
{
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

PubNav2GoalAction::PubNav2GoalAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = resolveNode(conf.blackboard);
  if (!node_) {
    throw BT::RuntimeError("PubNav2GoalAction blackboard entry 'node' is null");
  }

  auto port_it = conf.input_ports.find("topic_name");
  if (port_it != conf.input_ports.end()) {
    RCLCPP_INFO(
      logger_,
      "\033[1;92m[PubNav2Goal Constructor] input_port[topic_name] = %s\033[0m",
      port_it->second.c_str());
  }
}

rclcpp::Node::SharedPtr PubNav2GoalAction::resolveNode(BT::Blackboard::Ptr blackboard) const
{
  BT::Blackboard * current = blackboard.get();
  while (current) {
    auto any_locked = current->getAnyLocked("node");
    if (any_locked) {
      return any_locked->cast<rclcpp::Node::SharedPtr>();
    }
    current = current->parent().get();
  }

  throw BT::RuntimeError("PubNav2GoalAction requires blackboard entry 'node'");
}

bool PubNav2GoalAction::ensurePublisher(const std::string & topic_name)
{
  const std::string resolved_topic = topic_name.empty() ? "/goal_pose" : topic_name;
  if (publisher_ && prev_topic_name_ == resolved_topic) {
    return true;
  }

  publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(resolved_topic, rclcpp::QoS(1));
  prev_topic_name_ = resolved_topic;
  return true;
}

BT::NodeStatus PubNav2GoalAction::tick()
{
  auto goal = getInput<std::string>("goal");
  if (!goal) {
    RCLCPP_ERROR(logger_, "Missing required input port: goal");
    return BT::NodeStatus::FAILURE;
  }

  std::string topic_name = "/goal_pose";
  if (auto input_topic = getInput<std::string>("topic_name")) {
    topic_name = input_topic.value();
  }

  if (!ensurePublisher(topic_name)) {
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = "map";
  msg.pose = parseGoalString(*goal);

  publisher_->publish(msg);

  RCLCPP_INFO(
    logger_,
    "\033[1;92m[Action:PubNav2Goal] node=%s topic=%s goal_raw=%s pose=(%.3f, %.3f, %.3f) quat=(%.3f, %.3f, %.3f, %.3f)\033[0m",
    name().c_str(), prev_topic_name_.c_str(), goal->c_str(),
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList PubNav2GoalAction::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "goal", "0;0;0", "Expected goal pose that send to nav2. Fill with format `x;y;yaw`"),
    BT::InputPort<std::string>("topic_name", "/goal_pose", "Topic name used to publish nav goal"),
  };
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::PubNav2GoalAction>("PubNav2Goal");
}

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

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_NAV2_GOAL_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_NAV2_GOAL_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{
class PubNav2GoalAction : public BT::SyncActionNode
{
public:
  PubNav2GoalAction(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr resolveNode(BT::Blackboard::Ptr blackboard) const;
  bool ensurePublisher(const std::string & topic_name);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  std::string prev_topic_name_{"/goal_pose"};
  rclcpp::Logger logger_ = rclcpp::get_logger("PubNav2GoalAction");
};
}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_NAV2_GOAL_HPP_

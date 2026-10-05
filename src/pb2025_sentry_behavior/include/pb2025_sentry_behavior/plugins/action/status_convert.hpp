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

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__STATUS_CONVERT_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__STATUS_CONVERT_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace pb2025_sentry_behavior
{

class StatusConvertAction : public BT::StatefulActionNode
{
public:
  StatusConvertAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr resolveNode(BT::Blackboard::Ptr blackboard) const;
  bool publishStatus(int status);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::Logger logger_ = rclcpp::get_logger("StatusConvertAction");

  int original_status_{0};
  int temporary_status_{2};
  int hold_ms_{5000};
  std::string topic_name_{"/sentry_status"};
  bool temporary_status_published_{false};
  std::chrono::steady_clock::time_point restore_deadline_{};
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__STATUS_CONVERT_HPP_

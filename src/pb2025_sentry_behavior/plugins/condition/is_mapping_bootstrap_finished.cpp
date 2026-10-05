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

#include "pb2025_sentry_behavior/plugins/condition/is_mapping_bootstrap_finished.hpp"

namespace pb2025_sentry_behavior
{

namespace
{
constexpr const char * kLightGreen = "\033[1;92m";
constexpr const char * kLightBlue = "\033[1;94m";
constexpr const char * kColorReset = "\033[0m";

std::string greenLog(const std::string & message)
{
  return std::string(kLightGreen) + message + kColorReset;
}

std::string blueLog(const std::string & message)
{
  return std::string(kLightBlue) + message + kColorReset;
}
}  // namespace

IsMappingBootstrapFinishedCondition::IsMappingBootstrapFinishedCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(
    name, std::bind(&IsMappingBootstrapFinishedCondition::checkBootstrapStatus, this), config)
{
}

BT::NodeStatus IsMappingBootstrapFinishedCondition::checkBootstrapStatus()
{
  bool enabled = true;
  if (!getInput("enabled", enabled)) {
    RCLCPP_WARN(logger_, "enabled not provided, fallback to true");
  }

  if (!enabled) {
    RCLCPP_INFO(
      logger_, "%s",
      blueLog("[BT][MappingBootstrap] node=" + name() + " enabled=false result=SUCCESS").c_str());
    if (!last_gate_open_) {
      RCLCPP_INFO(logger_, "%s", greenLog("建图引导门控未启用，正常比赛行为树直接开始执行").c_str());
      last_gate_open_ = true;
    }
    return BT::NodeStatus::SUCCESS;
  }

  auto msg = getInput<std_msgs::msg::Bool>("key_port");
  if (!msg) {
    RCLCPP_WARN(logger_, "Mapping bootstrap flag is not available yet: %s", msg.error().c_str());
    RCLCPP_INFO(
      logger_, "%s",
      blueLog("[BT][MappingBootstrap] node=" + name() + " enabled=true flag_unavailable result=FAILURE")
        .c_str());
    last_gate_open_ = false;
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    logger_, "%s",
    blueLog(
      "[BT][MappingBootstrap] node=" + name() + " enabled=true finished=" +
      std::string(msg->data ? "true" : "false") + " result=" +
      std::string(msg->data ? "SUCCESS" : "FAILURE"))
      .c_str());

  if (msg->data && !last_gate_open_) {
    RCLCPP_INFO(logger_, "%s", greenLog("建图引导已完成，正常比赛行为树开始执行").c_str());
    last_gate_open_ = true;
  } else if (!msg->data) {
    last_gate_open_ = false;
  }

  return msg->data ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsMappingBootstrapFinishedCondition::providedPorts()
{
  return {
    BT::InputPort<bool>("enabled", true, "Whether bootstrap gating is enabled"),
    BT::InputPort<std_msgs::msg::Bool>(
      "key_port", "{@mapping_bootstrap_finished}", "Mapping bootstrap finished flag"),
  };
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsMappingBootstrapFinishedCondition>(
    "IsMappingBootstrapFinished");
}

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

#include "pb2025_sentry_behavior/plugins/condition/is_base_status_ok.hpp"

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
}  // namespace

IsBaseStatusOKCondition::IsBaseStatusOKCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsBaseStatusOKCondition::checkBaseStatus, this), config)
{
}

BT::NodeStatus IsBaseStatusOKCondition::checkBaseStatus()
{
  int base_status = 5000;
  int fortress_status = 2;

  auto msg = getInput<pb_rm_interfaces::msg::EventData>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "EventData message is not available: %s", msg.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("base_status", base_status)) {
    RCLCPP_WARN(logger_, "base_status not provided, fallback to default 5000");
  }
  if (!getInput("fortress_status", fortress_status)) {
    RCLCPP_WARN(logger_, "fortress_status not provided, fallback to default 2");
  }

  const bool is_base_low = (msg->base_hp < static_cast<uint16_t>(base_status));
  const bool is_fortress_enemy =
    (msg->fortress_gain_zone == msg->OCCUPIED_ENEMY) ||
    (msg->fortress_gain_zone == msg->OCCUPIED_BOTH) ||
    (msg->fortress_gain_zone == static_cast<uint8_t>(fortress_status));

  const auto result = (is_base_low || is_fortress_enemy) ? BT::NodeStatus::FAILURE
                                                          : BT::NodeStatus::SUCCESS;

  RCLCPP_INFO(
    logger_, "%s",
    blueLog(
      "[BT][BaseStatus] node=" + name() + " base_hp=" +
      std::to_string(static_cast<unsigned int>(msg->base_hp)) + "(th=" +
      std::to_string(base_status) + ") fortress_gain_zone=" +
      std::to_string(static_cast<unsigned int>(msg->fortress_gain_zone)) + " trigger=" +
      std::to_string(fortress_status) + " base_low=" +
      (is_base_low ? "true" : "false") + " fortress_danger=" +
      (is_fortress_enemy ? "true" : "false") + " result=" +
      (result == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE"))
      .c_str());

  return result;
}

BT::PortsList IsBaseStatusOKCondition::providedPorts()
{
  return {
    BT::InputPort<int>("base_status", 5000, "Base HP threshold. Lower than this returns FAILURE"),
    BT::InputPort<int>(
      "fortress_status", 2,
      "Fortress occupation status that should trigger FAILURE. Status 3 is always treated as danger"),
    BT::InputPort<pb_rm_interfaces::msg::EventData>(
      "key_port", "{@referee_eventData}", "EventData port on blackboard")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsBaseStatusOKCondition>("IsBaseStatusOK");
}

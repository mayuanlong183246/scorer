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

#include "pb2025_sentry_behavior/plugins/condition/is_status_time_ok.hpp"

namespace pb2025_sentry_behavior
{

IsStatusTimeOKCondition::IsStatusTimeOKCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsStatusTimeOKCondition::checkStatusTime, this), config)
{
}

void IsStatusTimeOKCondition::resetTimer()
{
  timer_started_ = false;
  last_status_ = std::numeric_limits<int>::min();
}

BT::NodeStatus IsStatusTimeOKCondition::checkStatusTime()
{
  int timeout_sec = 290;
  auto game_status_msg = getInput<pb_rm_interfaces::msg::GameStatus>("game_status_port");
  if (!game_status_msg) {
    RCLCPP_ERROR(logger_, "GameStatus message is not available: %s", game_status_msg.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto sentry_status_msg = getInput<std_msgs::msg::Int32>("status_port");
  if (!sentry_status_msg) {
    RCLCPP_ERROR(
      logger_, "Sentry status message is not available: %s", sentry_status_msg.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("timeout_sec", timeout_sec)) {
    RCLCPP_WARN(logger_, "timeout_sec not provided, fallback to default 290s");
  }

  const int current_status = sentry_status_msg->data;
  setOutput("current_status", current_status);

  if (game_status_msg->game_progress != 4) {
    resetTimer();
    setOutput("elapsed_sec", 0.0);
    return BT::NodeStatus::SUCCESS;
  }

  const auto now = std::chrono::steady_clock::now();
  if (!timer_started_) {
    timer_started_ = true;
    last_status_ = current_status;
    last_change_tp_ = now;
    setOutput("elapsed_sec", 0.0);
    RCLCPP_INFO(
      logger_, "[Condition:IsStatusTimeOK] node=%s start timing status=%d",
      name().c_str(), current_status);
    return BT::NodeStatus::SUCCESS;
  }

  if (current_status != last_status_) {
    RCLCPP_INFO(
      logger_, "[Condition:IsStatusTimeOK] node=%s status changed %d -> %d, timer reset",
      name().c_str(), last_status_, current_status);
    last_status_ = current_status;
    last_change_tp_ = now;
    setOutput("elapsed_sec", 0.0);
    return BT::NodeStatus::SUCCESS;
  }

  const double elapsed_sec =
    std::chrono::duration_cast<std::chrono::duration<double>>(now - last_change_tp_).count();
  setOutput("elapsed_sec", elapsed_sec);

  if (elapsed_sec > static_cast<double>(timeout_sec)) {
    RCLCPP_WARN(
      logger_,
      "[Condition:IsStatusTimeOK] node=%s status=%d elapsed=%.2f timeout=%d result=FAILURE",
      name().c_str(), current_status, elapsed_sec, timeout_sec);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

BT::PortsList IsStatusTimeOKCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::GameStatus>(
      "game_status_port", "{@referee_gameStatus}", "GameStatus port on blackboard"),
    BT::InputPort<std_msgs::msg::Int32>(
      "status_port", "{@sentry_status}", "Sentry status port on blackboard"),
    BT::InputPort<int>("timeout_sec", 290, "Maximum continuous dwell time in seconds"),
    BT::OutputPort<int>("current_status", "Current status when the timer is evaluated"),
    BT::OutputPort<double>("elapsed_sec", "Elapsed seconds for the current continuous status"),
  };
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsStatusTimeOKCondition>("IsStatusTimeOK");
}

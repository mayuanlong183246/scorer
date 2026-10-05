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

#include "pb2025_sentry_behavior/plugins/condition/is_game_status.hpp"

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

IsGameStatusCondition::IsGameStatusCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsGameStatusCondition::checkGameStart, this), config)
{
}

BT::NodeStatus IsGameStatusCondition::checkGameStart()
{
  int expected_game_progress = 4;
  int min_remain_time = 0;
  int max_remain_time = 420;
  getInput("expected_game_progress", expected_game_progress);
  getInput("min_remain_time", min_remain_time);
  getInput("max_remain_time", max_remain_time);

  auto msg = getInput<pb_rm_interfaces::msg::GameStatus>("key_port");
  if (!msg) {
    RCLCPP_WARN(
      logger_, "%s",
      blueLog(
        "[BT][GameStatus] node=" + name() + " message_unavailable=" + msg.error() +
        " expected_progress=" + std::to_string(expected_game_progress) + " remain_range=[" +
        std::to_string(min_remain_time) + "," + std::to_string(max_remain_time) + "] result=FAILURE")
        .c_str());
    return BT::NodeStatus::FAILURE;
  }

  const bool is_progress_match = (msg->game_progress == expected_game_progress);
  const bool is_time_in_range =
    (msg->stage_remain_time >= min_remain_time) && (msg->stage_remain_time <= max_remain_time);
  const auto result = (is_progress_match && is_time_in_range) ? BT::NodeStatus::SUCCESS
                                                               : BT::NodeStatus::FAILURE;

  RCLCPP_INFO(
    logger_, "%s",
    blueLog(
      "[BT][GameStatus] node=" + name() + " progress=" +
      std::to_string(static_cast<int>(msg->game_progress)) + "/" +
      std::to_string(expected_game_progress) + " remain=" +
      std::to_string(msg->stage_remain_time) + "s range=[" +
      std::to_string(min_remain_time) + "," + std::to_string(max_remain_time) + "] progress_ok=" +
      (is_progress_match ? "true" : "false") + " time_ok=" +
      (is_time_in_range ? "true" : "false") + " result=" +
      (result == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE"))
      .c_str());

  return result;
}

BT::PortsList IsGameStatusCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::GameStatus>(
      "key_port", "{@referee_gameStatus}", "GameStatus port on blackboard"),
    BT::InputPort<int>("expected_game_progress", 4, "Expected game progress stage"),
    BT::InputPort<int>("min_remain_time", 0, "Minimum remaining time (s)"),
    BT::InputPort<int>("max_remain_time", 420, "Maximum remaining time (s)"),
  };
}
}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsGameStatusCondition>("IsGameStatus");
}

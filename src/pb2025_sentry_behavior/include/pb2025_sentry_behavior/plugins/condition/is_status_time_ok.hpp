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

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_STATUS_TIME_OK_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_STATUS_TIME_OK_HPP_

#include <chrono>
#include <limits>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace pb2025_sentry_behavior
{

class IsStatusTimeOKCondition : public BT::SimpleConditionNode
{
public:
  IsStatusTimeOKCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus checkStatusTime();
  void resetTimer();

  bool timer_started_{false};
  int last_status_{std::numeric_limits<int>::min()};
  std::chrono::steady_clock::time_point last_change_tp_{};

  rclcpp::Logger logger_ = rclcpp::get_logger("IsStatusTimeOKCondition");
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_STATUS_TIME_OK_HPP_

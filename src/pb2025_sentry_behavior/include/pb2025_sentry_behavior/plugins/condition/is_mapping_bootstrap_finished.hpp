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

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_MAPPING_BOOTSTRAP_FINISHED_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_MAPPING_BOOTSTRAP_FINISHED_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace pb2025_sentry_behavior
{

class IsMappingBootstrapFinishedCondition : public BT::SimpleConditionNode
{
public:
  IsMappingBootstrapFinishedCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus checkBootstrapStatus();

  bool last_gate_open_{false};
  rclcpp::Logger logger_ = rclcpp::get_logger("IsMappingBootstrapFinishedCondition");
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_MAPPING_BOOTSTRAP_FINISHED_HPP_

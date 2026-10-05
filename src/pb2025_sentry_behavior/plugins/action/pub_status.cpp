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

#include "pb2025_sentry_behavior/plugins/action/pub_status.hpp"

namespace pb2025_sentry_behavior
{

PublishStatusAction::PublishStatusAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: RosTopicPubNode(name, config, params)
{
}

BT::PortsList PublishStatusAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<int>("sentry_status", 3, "Sentry status published directly to topic"),
  });
}

bool PublishStatusAction::setMessage(std_msgs::msg::Int32 & msg)
{
  int sentry_status = 3;
  getInput("sentry_status", sentry_status);
  std::string topic_name = "/sentry_status";
  getInput("topic_name", topic_name);

  msg.data = sentry_status;

  RCLCPP_INFO(
    node_->get_logger(),
    "[Action:PublishStatus] node=%s topic=%s sentry_status=%d",
    name().c_str(), topic_name.c_str(), msg.data);

  return true;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PublishStatusAction, "PublishStatus");

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

#include "pb2025_sentry_behavior/plugins/action/status_convert.hpp"

#include "behaviortree_cpp/basic_types.h"

namespace pb2025_sentry_behavior
{

StatusConvertAction::StatusConvertAction(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  node_ = resolveNode(config.blackboard);
  if (!node_) {
    throw BT::RuntimeError("StatusConvertAction blackboard entry 'node' is null");
  }
}

rclcpp::Node::SharedPtr StatusConvertAction::resolveNode(BT::Blackboard::Ptr blackboard) const
{
  BT::Blackboard * current = blackboard.get();
  while (current) {
    auto any_locked = current->getAnyLocked("node");
    if (any_locked) {
      return any_locked->cast<rclcpp::Node::SharedPtr>();
    }
    current = current->parent().get();
  }

  throw BT::RuntimeError("StatusConvertAction requires blackboard entry 'node'");
}

bool StatusConvertAction::publishStatus(int status)
{
  if (!publisher_) {
    publisher_ = node_->create_publisher<std_msgs::msg::Int32>(topic_name_, rclcpp::QoS(10));
  }

  std_msgs::msg::Int32 msg;
  msg.data = status;
  publisher_->publish(msg);

  RCLCPP_INFO(
    logger_, "[Action:StatusConvert] node=%s topic=%s publish_status=%d",
    name().c_str(), topic_name_.c_str(), status);
  return true;
}

BT::NodeStatus StatusConvertAction::onStart()
{
  auto original_status = getInput<int>("original_status");
  if (!original_status) {
    throw BT::RuntimeError(
            std::string("StatusConvertAction missing input [original_status]: ") +
            original_status.error());
  }

  original_status_ = original_status.value();
  temporary_status_ = 2;
  hold_ms_ = 5000;
  topic_name_ = "/sentry_status";

  if (auto temporary_status = getInput<int>("temporary_status")) {
    temporary_status_ = temporary_status.value();
  }
  if (auto hold_ms = getInput<int>("hold_ms")) {
    hold_ms_ = hold_ms.value();
  }
  if (auto topic_name = getInput<std::string>("topic_name")) {
    topic_name_ = topic_name.value();
  }

  if (original_status_ == temporary_status_) {
    RCLCPP_WARN(
      logger_,
      "[Action:StatusConvert] node=%s original_status equals temporary_status (%d), skip conversion",
      name().c_str(), original_status_);
    temporary_status_published_ = false;
    return BT::NodeStatus::SUCCESS;
  }

  publishStatus(temporary_status_);
  restore_deadline_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(hold_ms_);
  temporary_status_published_ = true;

  RCLCPP_INFO(
    logger_,
    "[Action:StatusConvert] node=%s original=%d temporary=%d hold_ms=%d result=RUNNING",
    name().c_str(), original_status_, temporary_status_, hold_ms_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StatusConvertAction::onRunning()
{
  if (!temporary_status_published_) {
    return BT::NodeStatus::SUCCESS;
  }

  if (std::chrono::steady_clock::now() < restore_deadline_) {
    return BT::NodeStatus::RUNNING;
  }

  publishStatus(original_status_);
  temporary_status_published_ = false;

  RCLCPP_INFO(
    logger_, "[Action:StatusConvert] node=%s restore original_status=%d result=SUCCESS",
    name().c_str(), original_status_);
  return BT::NodeStatus::SUCCESS;
}

void StatusConvertAction::onHalted()
{
  if (!temporary_status_published_) {
    return;
  }

  publishStatus(original_status_);
  temporary_status_published_ = false;
  RCLCPP_WARN(
    logger_, "[Action:StatusConvert] node=%s halted, restored original_status=%d",
    name().c_str(), original_status_);
}

BT::PortsList StatusConvertAction::providedPorts()
{
  return {
    BT::InputPort<int>("original_status", "Original status to restore after the temporary transition"),
    BT::InputPort<int>("temporary_status", 2, "Temporary status used to break continuous dwell timing"),
    BT::InputPort<int>("hold_ms", 5000, "How long to keep the temporary status in milliseconds"),
    BT::InputPort<std::string>("topic_name", "/sentry_status", "Topic used to publish the converted status"),
  };
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::StatusConvertAction>("StatusConvert");
}

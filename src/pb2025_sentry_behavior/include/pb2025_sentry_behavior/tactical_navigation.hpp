// Copyright 2026 RMUC contributors
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

#pragma once

#include <cmath>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <memory>
#include <string>

#include "behaviortree_cpp/basic_types.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

namespace pb2025_sentry_behavior
{
// One transport per tree. It remains alive and is polled even after a leaf is halted,
// so delayed goal acceptance and cancellation results cannot escape the handoff barrier.
class TacticalNavigation : public std::enable_shared_from_this<TacticalNavigation>
{
public:
  using Action = nav2_msgs::action::NavigateToPose;
  using Handle = rclcpp_action::ClientGoalHandle<Action>;
  explicit TacticalNavigation(const rclcpp::Node::SharedPtr & node) : node_(node)
  {
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    executor_.add_callback_group(group_, node_->get_node_base_interface());
    action_name_ = "navigate_to_pose";
    client_ = rclcpp_action::create_client<Action>(node_, action_name_, group_);
    diagnostics_ = node_->create_publisher<std_msgs::msg::String>("/tactical_navigation_status", 10);
    goals_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/tactical_goal", 10);
  }

  void poll()
  {
    // Humble actions multiplex feedback, status and service responses in one
    // waitable. spin_some services it only once per BT tick; frequent Nav2
    // feedback can starve goal acceptance/cancel responses at a 5 Hz tick.
    executor_.spin_all(std::chrono::milliseconds(5));
  }
  bool busy() const { return busy_; }
  bool succeeded() const { return success_; }
  const std::string & reason() const { return reason_; }
  bool cancelUnconfirmed(int timeout_ms) const
  {
    return busy_ && cancel_requested_ &&
           steadyNow() - cancel_stamp_ >= int64_t(timeout_ms) * 1000000;
  }

  bool start(const std::string & text, const std::string & action_name)
  {
    if (busy_) { return false; }
    success_ = false;
    reason_.clear();
    cancel_requested_ = false;
    cancel_sent_ = false;
    handle_.reset();
    if (!client_ || action_name != action_name_) {
      action_name_ = action_name;
      client_ = rclcpp_action::create_client<Action>(node_, action_name, group_);
    }
    if (!client_->action_server_is_ready()) {
      reason_ = "navigation_server_unavailable";
      return false;
    }
    Action::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = node_->now();
    const auto parts = BT::splitString(text, ';');
    try {
      if (parts.size() != 3 && parts.size() != 7) { throw std::runtime_error("goal size"); }
      double values[7]{};
      for (size_t i = 0; i < parts.size(); ++i) {
        values[i] = BT::convertFromString<double>(parts[i]);
        if (!std::isfinite(values[i])) { throw std::runtime_error("nonfinite goal"); }
      }
      auto & p = goal.pose.pose;
      p.position.x = values[0]; p.position.y = values[1];
      if (parts.size() == 3) {
        p.orientation.z = std::sin(values[2] / 2.0);
        p.orientation.w = std::cos(values[2] / 2.0);
      } else {
        p.position.z = values[2];
        const double norm = std::sqrt(values[3] * values[3] + values[4] * values[4] +
                                     values[5] * values[5] + values[6] * values[6]);
        if (norm < 1e-9) { throw std::runtime_error("zero quaternion"); }
        p.orientation.x = values[3] / norm; p.orientation.y = values[4] / norm;
        p.orientation.z = values[5] / norm; p.orientation.w = values[6] / norm;
      }
    } catch (const std::exception &) {
      reason_ = "invalid_goal";
      return false;
    }
    const auto generation = ++generation_;
    std::weak_ptr<TacticalNavigation> weak = shared_from_this();
    rclcpp_action::Client<Action>::SendGoalOptions options;
    options.goal_response_callback = [weak, generation](Handle::SharedPtr handle) {
      auto self = weak.lock();
      if (!self || generation != self->generation_) { return; }
      self->handle_ = handle;
      if (!handle) {
        self->busy_ = false;
        self->reason_ = "navigation_rejected";
      } else {
        self->report("accepted");
        if (self->cancel_requested_) { self->sendCancel(); }
      }
    };
    options.result_callback = [weak, generation](const Handle::WrappedResult & result) {
      auto self = weak.lock();
      if (!self || generation != self->generation_) { return; }
      self->busy_ = false;
      self->success_ = result.code == rclcpp_action::ResultCode::SUCCEEDED;
      self->reason_ = self->success_ ? "navigation_succeeded" :
        (result.code == rclcpp_action::ResultCode::CANCELED ? "navigation_canceled" :
         "navigation_aborted");
      self->report(self->reason_);
    };
    busy_ = true;
    try {
      client_->async_send_goal(goal, options);
    } catch (const std::exception &) {
      busy_ = false;
      reason_ = "navigation_send_failed";
      return false;
    }
    report("goal_sent");
    goals_->publish(goal.pose);
    return true;
  }

  void cancel()
  {
    if (!busy_ || cancel_requested_) { return; }
    cancel_requested_ = true;
    cancel_stamp_ = steadyNow();
    report("cancel_requested");
    sendCancel();
  }

private:
  void sendCancel()
  {
    if (!handle_ || cancel_sent_ || !busy_) { return; }
    cancel_sent_ = true;
    try {
      const auto generation = generation_;
      std::weak_ptr<TacticalNavigation> weak = shared_from_this();
      client_->async_cancel_goal(handle_, [weak, generation](auto response) {
        auto self = weak.lock();
        if (!self || generation != self->generation_) { return; }
        self->report("cancel_response code=" + std::to_string(response->return_code) +
          " goals=" + std::to_string(response->goals_canceling.size()));
      });
      report("cancel_sent");
    } catch (const std::exception & error) {
      reason_ = "navigation_cancel_unconfirmed";
      report(reason_ + ":" + error.what());
    }
    // A cancel acknowledgment alone is not terminal. Keep busy until the result
    // arrives; otherwise a new goal could overlap the old controller task.
  }
  static int64_t steadyNow()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void report(const std::string & state)
  {
    std::ostringstream uuid;
    if (handle_) {
      for (auto byte : handle_->get_goal_id()) {
        uuid << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
      }
    }
    std_msgs::msg::String message;
    message.data = "generation=" + std::to_string(generation_) + " uuid=" +
      (handle_ ? uuid.str() : "pending") + " state=" + state;
    diagnostics_->publish(message);
    RCLCPP_INFO(node_->get_logger(), "[Navigation] %s", message.data.c_str());
  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goals_;
  Handle::SharedPtr handle_;
  std::string action_name_, reason_;
  uint64_t generation_{0};
  int64_t cancel_stamp_{0};
  bool busy_{false}, success_{false}, cancel_requested_{false}, cancel_sent_{false};
};
}  // namespace pb2025_sentry_behavior

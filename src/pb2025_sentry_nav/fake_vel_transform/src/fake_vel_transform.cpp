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

#include "fake_vel_transform/fake_vel_transform.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

#include "fake_vel_transform/velocity_transform.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fake_vel_transform
{

FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "gimbal_link");
  fake_robot_base_frame_ =
    declare_parameter<std::string>("fake_robot_base_frame", "gimbal_link_fake");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "odom");
  cmd_spin_topic_ = declare_parameter<std::string>("cmd_spin_topic", "cmd_spin_transform");
  input_cmd_vel_topic_ =
    declare_parameter<std::string>("input_cmd_vel_topic", "cmd_vel_nav2_result");
  output_cmd_vel_topic_ =
    declare_parameter<std::string>("output_cmd_vel_topic", "cmd_vel_base_real_yaw");
  spin_speed_ = declare_parameter<double>("init_spin_speed", 0.0);
  command_timeout_ms_ = declare_parameter<double>("command_timeout_ms", 300.0);
  odom_timeout_ms_ = declare_parameter<double>("odom_timeout_ms", 500.0);
  publish_rate_ = declare_parameter<double>("publish_rate", 20.0);

  if (!std::isfinite(command_timeout_ms_) || command_timeout_ms_ <= 0.0 ||
    !std::isfinite(odom_timeout_ms_) || odom_timeout_ms_ <= 0.0 ||
    !std::isfinite(publish_rate_) || publish_rate_ <= 0.0)
  {
    throw std::invalid_argument("timeouts and publish_rate must be positive");
  }
  if (!std::isfinite(spin_speed_)) {
    throw std::invalid_argument("init_spin_speed must be finite");
  }

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  cmd_vel_chassis_pub_ =
    create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, command_qos);
  cmd_spin_sub_ = create_subscription<example_interfaces::msg::Float32>(
    cmd_spin_topic_, command_qos,
    std::bind(&FakeVelTransform::cmdSpinCallback, this, std::placeholders::_1));
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, command_qos,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&FakeVelTransform::odometryCallback, this, std::placeholders::_1));

  timer_ = create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&FakeVelTransform::publishTransform, this));
  const auto velocity_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_));
  velocity_timer_ = create_wall_timer(
    velocity_period, std::bind(&FakeVelTransform::publishVelocity, this));

  RCLCPP_INFO(
    get_logger(),
    "Velocity transform %s -> %s (command timeout %.0f ms, odom timeout %.0f ms, %.1f Hz)",
    input_cmd_vel_topic_.c_str(), output_cmd_vel_topic_.c_str(), command_timeout_ms_,
    odom_timeout_ms_, publish_rate_);
}

void FakeVelTransform::cmdSpinCallback(const example_interfaces::msg::Float32::SharedPtr msg)
{
  if (!std::isfinite(msg->data)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Ignoring non-finite spin command");
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  spin_speed_ = msg->data;
}

void FakeVelTransform::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  const double yaw = tf2::getYaw(msg->pose.pose.orientation);
  if (!std::isfinite(yaw)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring invalid odometry yaw");
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  current_robot_base_angle_ = yaw;
  last_odom_time_ = std::chrono::steady_clock::now();
  have_odom_ = true;
}

void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!isFiniteTwist(*msg)) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    have_command_ = false;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Rejecting non-finite command velocity");
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_cmd_vel_ = *msg;
  last_command_time_ = std::chrono::steady_clock::now();
  have_command_ = true;
}

void FakeVelTransform::publishVelocity()
{
  const auto current = std::chrono::steady_clock::now();
  geometry_msgs::msg::Twist command;
  double yaw = 0.0;
  double spin_speed = 0.0;
  double command_age = std::numeric_limits<double>::infinity();
  double odom_age = std::numeric_limits<double>::infinity();
  bool have_command = false;
  bool have_odom = false;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    command = latest_cmd_vel_;
    yaw = current_robot_base_angle_;
    spin_speed = spin_speed_;
    have_command = have_command_;
    have_odom = have_odom_;
    if (have_command) {
      command_age =
        std::chrono::duration<double, std::milli>(current - last_command_time_).count();
    }
    if (have_odom) {
      odom_age = std::chrono::duration<double, std::milli>(current - last_odom_time_).count();
    }
  }

  const bool command_fresh = have_command && command_age <= command_timeout_ms_;
  const bool odom_fresh = have_odom && odom_age <= odom_timeout_ms_;
  if (command_fresh && odom_fresh) {
    cmd_vel_chassis_pub_->publish(transformVelocity(command, yaw, spin_speed));
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Published converted velocity: command age %.0f ms, odom age %.0f ms, yaw %.3f",
      command_age, odom_age, yaw);
    return;
  }

  cmd_vel_chassis_pub_->publish(geometry_msgs::msg::Twist{});
  const char * reason = !command_fresh ? "command missing or stale" : "odometry missing or stale";
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "Publishing zero velocity: %s (command age %.0f ms, odom age %.0f ms)",
    reason, command_age, odom_age);
}

void FakeVelTransform::publishTransform()
{
  double yaw = 0.0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    yaw = current_robot_base_angle_;
  }

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = now();
  transform.header.frame_id = robot_base_frame_;
  transform.child_frame_id = fake_robot_base_frame_;
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, -yaw);
  transform.transform.rotation = tf2::toMsg(quaternion);
  tf_broadcaster_->sendTransform(transform);
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)

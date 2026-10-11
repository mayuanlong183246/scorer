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

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>

#include "fake_vel_transform/fake_vel_transform.hpp"
#include "fake_vel_transform/velocity_transform.hpp"

using namespace std::chrono_literals;

namespace fake_vel_transform
{

TEST(VelocityTransform, RotatesAndAddsSpin)
{
  geometry_msgs::msg::Twist input;
  input.linear.x = 1.0;
  input.angular.z = 0.25;

  const auto result = transformVelocity(input, M_PI_2, 0.5);

  EXPECT_NEAR(result.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(result.linear.y, -1.0, 1e-9);
  EXPECT_DOUBLE_EQ(result.angular.z, 0.75);
}

TEST(VelocityTransform, RejectsNonFiniteComponent)
{
  geometry_msgs::msg::Twist input;
  EXPECT_TRUE(isFiniteTwist(input));
  input.linear.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(isFiniteTwist(input));
}

class VelocityPipelineTest : public testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("input_cmd_vel_topic", "/test_velocity/input"),
      rclcpp::Parameter("output_cmd_vel_topic", "/test_velocity/output"),
      rclcpp::Parameter("odom_topic", "/test_velocity/odom"),
      rclcpp::Parameter("cmd_spin_topic", "/test_velocity/spin"),
      rclcpp::Parameter("command_timeout_ms", 120.0),
      rclcpp::Parameter("odom_timeout_ms", 120.0),
      rclcpp::Parameter("publish_rate", 50.0),
    });
    transform_node_ = std::make_shared<FakeVelTransform>(options);
    test_node_ = std::make_shared<rclcpp::Node>("fake_vel_transform_test_client");

    command_pub_ = test_node_->create_publisher<geometry_msgs::msg::Twist>(
      "/test_velocity/input", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    odom_pub_ = test_node_->create_publisher<nav_msgs::msg::Odometry>(
      "/test_velocity/odom", rclcpp::SensorDataQoS().keep_last(1));
    output_sub_ = test_node_->create_subscription<geometry_msgs::msg::Twist>(
      "/test_velocity/output", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(output_mutex_);
        latest_output_ = *msg;
        ++output_count_;
      });

    executor_.add_node(transform_node_);
    executor_.add_node(test_node_);
    spin_thread_ = std::thread([this]() {executor_.spin();});
    ASSERT_TRUE(waitFor([this]() {
      return command_pub_->get_subscription_count() == 1U &&
             odom_pub_->get_subscription_count() == 1U;
    }, 2s));
  }

  void TearDown() override
  {
    executor_.cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_.remove_node(test_node_);
    executor_.remove_node(transform_node_);
    output_sub_.reset();
    command_pub_.reset();
    odom_pub_.reset();
    test_node_.reset();
    transform_node_.reset();
  }

  template<typename PredicateT>
  bool waitFor(PredicateT predicate, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return predicate();
  }

  geometry_msgs::msg::Twist latestOutput()
  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    return latest_output_;
  }

  std::size_t outputCount()
  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    return output_count_;
  }

  std::shared_ptr<FakeVelTransform> transform_node_;
  rclcpp::Node::SharedPtr test_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr output_sub_;
  std::mutex output_mutex_;
  geometry_msgs::msg::Twist latest_output_;
  std::size_t output_count_{0};
};

TEST_F(VelocityPipelineTest, PublishesWithoutLocalPlanAndFailsSafeOnStaleData)
{
  ASSERT_TRUE(waitFor([this]() {return outputCount() > 0U;}, 1s));
  EXPECT_DOUBLE_EQ(latestOutput().linear.x, 0.0);

  nav_msgs::msg::Odometry odom;
  odom.pose.pose.orientation.z = std::sqrt(0.5);
  odom.pose.pose.orientation.w = std::sqrt(0.5);
  geometry_msgs::msg::Twist command;
  command.linear.x = 1.0;
  command.angular.z = 0.25;
  odom_pub_->publish(odom);
  command_pub_->publish(command);

  ASSERT_TRUE(waitFor([this]() {
    return std::abs(latestOutput().linear.y + 1.0) < 1e-6;
  }, 1s));
  EXPECT_NEAR(latestOutput().angular.z, 0.25, 1e-9);

  ASSERT_TRUE(waitFor([this]() {
    const auto output = latestOutput();
    return output.linear.x == 0.0 && output.linear.y == 0.0 && output.angular.z == 0.0;
  }, 1s));

  command_pub_->publish(command);
  std::this_thread::sleep_for(50ms);
  EXPECT_DOUBLE_EQ(latestOutput().linear.y, 0.0);

  odom_pub_->publish(odom);
  command_pub_->publish(command);
  EXPECT_TRUE(waitFor([this]() {
    return std::abs(latestOutput().linear.y + 1.0) < 1e-6;
  }, 1s));
}

}  // namespace fake_vel_transform

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

#ifndef FAKE_VEL_TRANSFORM__VELOCITY_TRANSFORM_HPP_
#define FAKE_VEL_TRANSFORM__VELOCITY_TRANSFORM_HPP_

#include <cmath>

#include "geometry_msgs/msg/twist.hpp"

namespace fake_vel_transform
{

inline bool isFiniteTwist(const geometry_msgs::msg::Twist & twist)
{
  const double values[] = {
    twist.linear.x, twist.linear.y, twist.linear.z,
    twist.angular.x, twist.angular.y, twist.angular.z};
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

inline geometry_msgs::msg::Twist transformVelocity(
  const geometry_msgs::msg::Twist & twist, const double yaw, const double spin_speed)
{
  geometry_msgs::msg::Twist result;
  result.linear.x = twist.linear.x * std::cos(yaw) + twist.linear.y * std::sin(yaw);
  result.linear.y = -twist.linear.x * std::sin(yaw) + twist.linear.y * std::cos(yaw);
  result.angular.z = twist.angular.z + spin_speed;
  return result;
}

}  // namespace fake_vel_transform

#endif  // FAKE_VEL_TRANSFORM__VELOCITY_TRANSFORM_HPP_

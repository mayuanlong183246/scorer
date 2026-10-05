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

#include "pb2025_sentry_behavior/pb2025_sentry_behavior_server.hpp"

#include <filesystem>
#include <fstream>

#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "behaviortree_cpp/xml_parsing.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pb_rm_interfaces/msg/buff.hpp"
#include "pb_rm_interfaces/msg/event_data.hpp"
#include "pb_rm_interfaces/msg/game_robot_hp.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/ground_robot_position.hpp"
#include "pb_rm_interfaces/msg/rfid_status.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "pb2025_sentry_behavior/custom_types.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
namespace pb2025_sentry_behavior
{

namespace
{
constexpr const char * kLightGreen = "\033[1;92m";
constexpr const char * kColorReset = "\033[0m";

std::string greenLog(const std::string & message)
{
  return std::string(kLightGreen) + message + kColorReset;
}
}  // namespace

template <typename T>
void SentryBehaviorServer::subscribe(
  const std::string & topic, const std::string & bb_key, const rclcpp::QoS & qos)
{
  auto sub = node()->create_subscription<T>(
    topic, qos,
    [this, bb_key](const typename T::SharedPtr msg) {
      globalBlackboard()->set(bb_key, *msg);
      globalBlackboard()->set(bb_key + "_stamp_ns", node()->now().nanoseconds());
    });
  subscriptions_.push_back(sub);
}

SentryBehaviorServer::SentryBehaviorServer(const rclcpp::NodeOptions & options)
: TreeExecutionServer(options),
  last_tactical_decision_stamp_(0, 0, RCL_ROS_TIME)
{
  globalBlackboard()->set("node", node());
  globalBlackboard()->set("validate_navigation_goals",
    node()->declare_parameter("validate_navigation_goals", false));
  subscribe<nav_msgs::msg::OccupancyGrid>("map", "tactical_map",
    rclcpp::QoS(1).transient_local().reliable());
  globalBlackboard()->set("status_convert_temp_status", 2);
  node()->declare_parameter("mapping_bootstrap_enabled", false);
  bool mapping_bootstrap_enabled = false;
  node()->get_parameter("mapping_bootstrap_enabled", mapping_bootstrap_enabled);
  globalBlackboard()->set("mapping_bootstrap_enabled", mapping_bootstrap_enabled);
  globalBlackboard()->set("mapping_bootstrap_finished", std_msgs::msg::Bool());
  globalBlackboard()->set(
    "tactical_decision",
    pb2025_sentry_decision_interfaces::msg::TacticalDecision());
  globalBlackboard()->set("tactical_decision_stamp_ns", int64_t{0});
  globalBlackboard()->set("active_tactical_action", -1);

  node()->declare_parameter("tactical_decision_topic", "/tactical_decision");
  node()->declare_parameter("decision_timeout_ms", 1000);
  node()->declare_parameter("team_color", "blue");
  node()->declare_parameter("sentry_robot_id", 7);
  node()->declare_parameter("teammate_data_timeout_ms", 500);
  node()->declare_parameter("cover_teammate_hp_threshold", 250);
  node()->declare_parameter("outpost_alive_min_hp", 1);
  node()->declare_parameter("cover_home_reference_x", 0.0);
  node()->declare_parameter("cover_home_reference_y", 0.0);
  node()->declare_parameter("cover_rear_distance", 1.5);
  node()->declare_parameter("cover_side_offset", 1.0);
  node()->declare_parameter("cover_side_sign", 1.0);
  node()->declare_parameter("outpost_attack_goal", "5.70;-2.93;0.0");
  node()->declare_parameter("resupply_goal", "-0.90;-6.00;0.0");
  node()->declare_parameter("defend_base_goal", "2.70;-0.53;0.0");
  node()->declare_parameter("defend_fortress_goal", "4.80;-2.00;0.0");
  node()->declare_parameter("capture_point_a_goal", "8.65;-3.50;0.0");
  node()->declare_parameter("capture_point_b_goal", "4.00;3.00;0.0");
  node()->declare_parameter("capture_point_a_dwell_ms", 10000);
  node()->declare_parameter("capture_point_b_dwell_ms", 10000);
  node()->declare_parameter("gather_info_goal", "5.42;-3.41;0.0");
  node()->declare_parameter("gather_info_dwell_ms", 5000);
  const std::vector<std::pair<std::string, int>> tactical_parameters = {
    {"navigation_timeout_ms", 60000}, {"navigation_cancel_timeout_ms", 2000},
    {"action_failure_cooldown_ms", 5000}, {"cover_dwell_ms", 10000},
    {"outpost_attack_timeout_ms", 60000}, {"resupply_timeout_ms", 30000},
    {"resupply_hp_target", 400}, {"resupply_ammo_target", 150},
    {"defend_dwell_ms", 10000}, {"target_locked_timeout_ms", 1000},
    {"referee_data_timeout_ms", 1000}, {"base_hp_low", 5000}};
  for (const auto & parameter : tactical_parameters) {
    const int value = node()->declare_parameter<int>(parameter.first, parameter.second);
    if (value <= 0) { throw std::invalid_argument(parameter.first + " must be positive"); }
    globalBlackboard()->set(parameter.first, value);
  }
  const auto target_locked_topic = node()->declare_parameter<std::string>(
    "target_locked_topic", "/auto_aim/target_locked");
  subscribe<std_msgs::msg::Bool>(target_locked_topic, "auto_aim_target_locked");

  std::string tactical_decision_topic;
  node()->get_parameter("tactical_decision_topic", tactical_decision_topic);
  int decision_timeout_ms = 1000;
  node()->get_parameter("decision_timeout_ms", decision_timeout_ms);
  std::string team_color;
  node()->get_parameter("team_color", team_color);
  int sentry_robot_id = 7;
  node()->get_parameter("sentry_robot_id", sentry_robot_id);
  int teammate_data_timeout_ms = 500;
  node()->get_parameter("teammate_data_timeout_ms", teammate_data_timeout_ms);
  int cover_teammate_hp_threshold = 250;
  node()->get_parameter("cover_teammate_hp_threshold", cover_teammate_hp_threshold);
  int outpost_alive_min_hp = 1;
  node()->get_parameter("outpost_alive_min_hp", outpost_alive_min_hp);
  double cover_home_reference_x = 0.0;
  double cover_home_reference_y = 0.0;
  double cover_rear_distance = 1.5;
  double cover_side_offset = 1.0;
  double cover_side_sign = 1.0;
  node()->get_parameter("cover_home_reference_x", cover_home_reference_x);
  node()->get_parameter("cover_home_reference_y", cover_home_reference_y);
  node()->get_parameter("cover_rear_distance", cover_rear_distance);
  node()->get_parameter("cover_side_offset", cover_side_offset);
  node()->get_parameter("cover_side_sign", cover_side_sign);

  globalBlackboard()->set("decision_timeout_ms", decision_timeout_ms);
  globalBlackboard()->set("team_color", team_color);
  globalBlackboard()->set("sentry_robot_id", sentry_robot_id);
  globalBlackboard()->set("teammate_data_timeout_ms", teammate_data_timeout_ms);
  globalBlackboard()->set("cover_teammate_hp_threshold", cover_teammate_hp_threshold);
  globalBlackboard()->set("outpost_alive_min_hp", outpost_alive_min_hp);
  globalBlackboard()->set("cover_home_reference_x", cover_home_reference_x);
  globalBlackboard()->set("cover_home_reference_y", cover_home_reference_y);
  globalBlackboard()->set("cover_rear_distance", cover_rear_distance);
  globalBlackboard()->set("cover_side_offset", cover_side_offset);
  globalBlackboard()->set("cover_side_sign", cover_side_sign);
  const std::vector<std::string> goal_parameter_names = {
    "outpost_attack_goal", "resupply_goal", "defend_base_goal", "defend_fortress_goal",
    "capture_point_a_goal", "capture_point_b_goal", "gather_info_goal"};
  for (const auto & name : goal_parameter_names) {
    std::string value;
    node()->get_parameter(name, value);
    globalBlackboard()->set(name, value);
  }
  const std::vector<std::string> duration_parameter_names = {
    "capture_point_a_dwell_ms", "capture_point_b_dwell_ms", "gather_info_dwell_ms"};
  for (const auto & name : duration_parameter_names) {
    int value = 0;
    node()->get_parameter(name, value);
    globalBlackboard()->set(name, std::max(value, 0));
  }
  RCLCPP_INFO(
    node()->get_logger(), "%s",
    greenLog(std::string("行为树建图引导门控: ") +
    (mapping_bootstrap_enabled ? "启用" : "关闭")).c_str());

  node()->declare_parameter("use_cout_logger", false);
  node()->get_parameter("use_cout_logger", use_cout_logger_);

  subscribe<pb_rm_interfaces::msg::EventData>("referee/event_data", "referee_eventData");
  subscribe<pb_rm_interfaces::msg::GameRobotHP>("referee/all_robot_hp", "referee_allRobotHP");
  subscribe<pb_rm_interfaces::msg::GameStatus>("referee/game_status", "referee_gameStatus");
  subscribe<pb_rm_interfaces::msg::GroundRobotPosition>(
    "referee/ground_robot_position", "referee_groundRobotPosition");
  subscribe<pb_rm_interfaces::msg::RfidStatus>("referee/rfid_status", "referee_rfidStatus");
  subscribe<pb_rm_interfaces::msg::RobotStatus>("referee/robot_status", "referee_robotStatus");
  subscribe<pb_rm_interfaces::msg::Buff>("referee/buff", "referee_buff");
  subscribe<std_msgs::msg::Int32>("/sentry_status", "sentry_status");
  subscribe<std_msgs::msg::Bool>("/mapping_bootstrap_finished", "mapping_bootstrap_finished",
    rclcpp::QoS(1).transient_local().reliable());

  auto detector_qos = rclcpp::SensorDataQoS();
  subscribe<auto_aim_interfaces::msg::Armors>("detector/armors", "detector_armors", detector_qos);
  auto tracker_qos = rclcpp::SensorDataQoS();
  subscribe<auto_aim_interfaces::msg::Target>("tracker/target", "tracker_target", tracker_qos);
  subscribe<pb2025_sentry_decision_interfaces::msg::TacticalDecision>(
    tactical_decision_topic, "tactical_decision");

  auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  subscribe<nav_msgs::msg::OccupancyGrid>(
    "global_costmap/costmap", "nav_globalCostmap", costmap_qos);
}

bool SentryBehaviorServer::onGoalReceived(
  const std::string & tree_name, const std::string & payload)
{
  RCLCPP_INFO(
    node()->get_logger(), "onGoalReceived with tree name '%s' with payload '%s'", tree_name.c_str(),
    payload.c_str());
  return true;
}

void SentryBehaviorServer::onTreeCreated(BT::Tree & tree)
{
  if (use_cout_logger_) {
    logger_cout_ = std::make_shared<BT::StdCoutLogger>(tree);
  }
  tick_count_ = 0;
}

std::optional<BT::NodeStatus> SentryBehaviorServer::onLoopAfterTick(BT::NodeStatus /*status*/)
{
  ++tick_count_;
  return std::nullopt;
}

std::optional<std::string> SentryBehaviorServer::onTreeExecutionCompleted(
  BT::NodeStatus status, bool was_cancelled)
{
  RCLCPP_INFO(
    node()->get_logger(), "onTreeExecutionCompleted with status=%d (canceled=%d) after %d ticks",
    static_cast<int>(status), was_cancelled, tick_count_);
  logger_cout_.reset();

  return was_cancelled ? "behavior_tree_cancelled" : "behavior_tree_completed";
}

}  // namespace pb2025_sentry_behavior

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto action_server = std::make_shared<pb2025_sentry_behavior::SentryBehaviorServer>(options);

  RCLCPP_INFO(action_server->node()->get_logger(), "Starting SentryBehaviorServer");

  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
  exec.add_node(action_server->node());
  exec.spin();
  exec.remove_node(action_server->node());

  rclcpp::shutdown();
}

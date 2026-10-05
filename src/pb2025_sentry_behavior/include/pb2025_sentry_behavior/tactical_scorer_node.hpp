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

#ifndef PB2025_SENTRY_BEHAVIOR__TACTICAL_SCORER_NODE_HPP_
#define PB2025_SENTRY_BEHAVIOR__TACTICAL_SCORER_NODE_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include "auto_aim_interfaces/msg/target.hpp"
#include "std_msgs/msg/bool.hpp"
#include "pb2025_sentry_decision_interfaces/msg/tactical_execution.hpp"
#include "pb2025_sentry_decision_interfaces/msg/tactical_decision.hpp"
#include "pb_rm_interfaces/msg/event_data.hpp"
#include "pb_rm_interfaces/msg/game_robot_hp.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/ground_robot_position.hpp"
#include "pb_rm_interfaces/msg/rfid_status.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace pb2025_sentry_behavior
{

class TacticalScorerNode : public rclcpp::Node
{
public:
  explicit TacticalScorerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using TacticalDecision = pb2025_sentry_decision_interfaces::msg::TacticalDecision;

  struct DecisionDraft
  {
    uint8_t action;
    std::string action_name;
    float score;
    std::vector<std::string> top_reasons;
    std::vector<std::string> rejected_actions;
    bool emergency;
    std::string target_id;
  };

  void onTimer();
  void onSimulationState(const std_msgs::msg::String::SharedPtr msg);
  DecisionDraft score();
  TacticalDecision makeDecisionMsg(const DecisionDraft & draft);
  bool isTargetFresh() const;
  bool isRobotResourceLow() const;
  bool isRobotResourceCritical() const;
  bool isFresh(const rclcpp::Time & stamp, int timeout_ms) const;
  bool isBaseDanger() const;
  bool isCapturePointInteresting() const;
  double coverUrgency() const;
  bool isOutpostAlive() const;
  uint16_t teammateHp(uint8_t robot_id) const;
  std::string actionName(uint8_t action) const;

  double tick_hz_;
  int hp_low_;
  int ammo_low_;
  int hp_critical_;
  int ammo_critical_;
  double heat_low_ratio_;
  double heat_critical_ratio_;
  int referee_data_timeout_ms_;
  bool mapping_bootstrap_enabled_{false};
  bool mapping_finished_{false};
  int failure_cooldown_ms_{5000};
  std::array<int64_t, 7> cooldown_until_{};
  std::unordered_set<uint32_t> failed_decisions_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mapping_sub_;
  rclcpp::Subscription<pb2025_sentry_decision_interfaces::msg::TacticalExecution>::SharedPtr
    execution_sub_;
  bool available(uint8_t action) const;
  int base_hp_low_;
  int target_lost_timeout_ms_;
  int sentry_robot_id_;
  int teammate_data_timeout_ms_;
  int cover_teammate_hp_threshold_;
  int outpost_alive_min_hp_;
  int outpost_reference_hp_;
  std::string team_color_;
  std::string decision_topic_;

  uint32_t decision_id_;
  uint32_t world_state_id_{0};
  int decision_hold_ms_{500};
  int decision_timeout_ms_{1000};
  float action_switch_margin_{0.08F};
  uint8_t last_action_{255};
  rclcpp::Time last_action_change_;

  std::optional<pb_rm_interfaces::msg::GameStatus> game_status_;
  std::optional<pb_rm_interfaces::msg::RobotStatus> robot_status_;
  std::optional<pb_rm_interfaces::msg::GameRobotHP> all_robot_hp_;
  std::optional<pb_rm_interfaces::msg::GroundRobotPosition> ground_robot_position_;
  std::optional<pb_rm_interfaces::msg::EventData> event_data_;
  std::optional<pb_rm_interfaces::msg::RfidStatus> rfid_status_;
  std::optional<auto_aim_interfaces::msg::Target> tracker_target_;
  rclcpp::Time last_target_stamp_;
  rclcpp::Time last_game_status_stamp_;
  rclcpp::Time last_robot_status_stamp_;
  rclcpp::Time last_event_data_stamp_;

  std::array<float, 7> action_importance_{};
  int defense_threat_observations_{0};

  rclcpp::Subscription<pb_rm_interfaces::msg::GameStatus>::SharedPtr game_status_sub_;
  rclcpp::Subscription<pb_rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_sub_;
  rclcpp::Subscription<pb_rm_interfaces::msg::GameRobotHP>::SharedPtr all_robot_hp_sub_;
  rclcpp::Subscription<pb_rm_interfaces::msg::GroundRobotPosition>::SharedPtr
    ground_robot_position_sub_;
  rclcpp::Subscription<pb_rm_interfaces::msg::EventData>::SharedPtr event_data_sub_;
  rclcpp::Subscription<pb_rm_interfaces::msg::RfidStatus>::SharedPtr rfid_status_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr tracker_target_sub_;
  rclcpp::Publisher<TacticalDecision>::SharedPtr decision_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr simulation_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_all_robot_hp_stamp_;
  rclcpp::Time last_ground_robot_position_stamp_;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__TACTICAL_SCORER_NODE_HPP_

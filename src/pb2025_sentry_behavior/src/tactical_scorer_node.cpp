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

#include "pb2025_sentry_behavior/tactical_scorer_node.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace pb2025_sentry_behavior
{

namespace
{
constexpr uint8_t kHoldSafe =
  pb2025_sentry_decision_interfaces::msg::TacticalDecision::HOLD_SAFE;
constexpr uint8_t kCoverTeammate =
  pb2025_sentry_decision_interfaces::msg::TacticalDecision::COVER_TEAMMATE;
constexpr uint8_t kAttackOutpost =
  pb2025_sentry_decision_interfaces::msg::TacticalDecision::ATTACK_OUTPOST;
constexpr uint8_t kResupplyHome =
  pb2025_sentry_decision_interfaces::msg::TacticalDecision::RESUPPLY_HOME;
constexpr uint8_t kDefendHome =
  pb2025_sentry_decision_interfaces::msg::TacticalDecision::DEFEND_HOME;
constexpr uint8_t kCapturePoint =
  pb2025_sentry_decision_interfaces::msg::TacticalDecision::CAPTURE_POINT;
constexpr uint8_t kGatherInfo =
  pb2025_sentry_decision_interfaces::msg::TacticalDecision::GATHER_INFO;

double linear(double value, double low, double high)
{
  return std::clamp((value - low) / (high - low), 0.0, 1.0);
}
}  // namespace

TacticalScorerNode::TacticalScorerNode(const rclcpp::NodeOptions & options)
: Node("tactical_scorer_node", options),
  decision_id_(0),
  last_action_change_(0, 0, get_clock()->get_clock_type()),
  last_target_stamp_(0, 0, get_clock()->get_clock_type()),
  last_game_status_stamp_(0, 0, get_clock()->get_clock_type()),
  last_robot_status_stamp_(0, 0, get_clock()->get_clock_type()),
  last_event_data_stamp_(0, 0, get_clock()->get_clock_type()),
  last_all_robot_hp_stamp_(0, 0, get_clock()->get_clock_type()),
  last_ground_robot_position_stamp_(0, 0, get_clock()->get_clock_type())
{
  tick_hz_ = declare_parameter<double>("tick_hz", 5.0);
  hp_low_ = declare_parameter<int>("hp_low", 180);
  ammo_low_ = declare_parameter<int>("ammo_low", 50);
  hp_critical_ = declare_parameter<int>("hp_critical", 100);
  ammo_critical_ = declare_parameter<int>("ammo_critical", 10);
  heat_low_ratio_ = declare_parameter<double>("heat_low_ratio", 0.80);
  heat_critical_ratio_ = declare_parameter<double>("heat_critical_ratio", 0.95);
  referee_data_timeout_ms_ = declare_parameter<int>("referee_data_timeout_ms", 1000);
  mapping_bootstrap_enabled_ = declare_parameter<bool>("mapping_bootstrap_enabled", false);
  failure_cooldown_ms_ = declare_parameter<int>("action_failure_cooldown_ms", 5000);
  mapping_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/mapping_bootstrap_finished", rclcpp::QoS(1).transient_local().reliable(),
    [this](const std_msgs::msg::Bool::SharedPtr msg) { mapping_finished_ = msg->data; });
  execution_sub_ = create_subscription<pb2025_sentry_decision_interfaces::msg::TacticalExecution>(
    "/tactical_execution", 10,
    [this](const pb2025_sentry_decision_interfaces::msg::TacticalExecution::SharedPtr msg) {
      if (msg->state == "FAILED" && msg->action > 0 && msg->action < 7 &&
          failed_decisions_.insert(msg->decision_id).second) {
        cooldown_until_[msg->action] = now().nanoseconds() +
          int64_t(std::max(0, failure_cooldown_ms_)) * 1000000;
      }
    });
  base_hp_low_ = declare_parameter<int>("base_hp_low", 5000);
  target_lost_timeout_ms_ = declare_parameter<int>("target_lost_timeout_ms", 800);
  team_color_ = declare_parameter<std::string>("team_color", "blue");
  sentry_robot_id_ = declare_parameter<int>("sentry_robot_id", 7);
  teammate_data_timeout_ms_ = declare_parameter<int>("teammate_data_timeout_ms", 500);
  cover_teammate_hp_threshold_ = declare_parameter<int>("cover_teammate_hp_threshold", 250);
  outpost_alive_min_hp_ = declare_parameter<int>("outpost_alive_min_hp", 1);
  outpost_reference_hp_ = declare_parameter<int>("outpost_reference_hp", 1800);
  decision_hold_ms_ = declare_parameter<int>("decision_hold_ms", 500);
  decision_timeout_ms_ = declare_parameter<int>("decision_timeout_ms", 1000);
  action_switch_margin_ = declare_parameter<float>("action_switch_margin", 0.08F);
  decision_topic_ = declare_parameter<std::string>("decision_topic", "/tactical_decision");

  const auto load_importance = [this](const std::string & name, const std::array<float, 7> & defaults) {
      const auto values = declare_parameter<std::vector<double>>(
        name, std::vector<double>(defaults.begin(), defaults.end()));
      std::array<float, 7> result = defaults;
      if (values.size() == result.size()) {
        for (std::size_t i = 0; i < result.size(); ++i) {
          if (!std::isfinite(values[i]) || values[i] < 0.0 || values[i] > 1.0) {
            throw std::invalid_argument(name + " must contain finite weights in [0,1]");
          }
          result[i] = static_cast<float>(values[i]);
        }
      } else {
        throw std::invalid_argument(name + " must contain exactly 7 weights");
      }
      return result;
    };
  action_importance_ = load_importance(
    "action_importance", {0.05F, 0.72F, 0.78F, 0.90F, 0.82F, 0.62F, 0.58F});
  if (hp_critical_ < 0 || hp_low_ <= hp_critical_ || hp_low_ > 65535 ||
      ammo_critical_ < 0 || ammo_low_ <= ammo_critical_ || ammo_low_ > 65535 ||
      !std::isfinite(heat_low_ratio_) || !std::isfinite(heat_critical_ratio_) ||
      heat_low_ratio_ < 0.0 || heat_low_ratio_ >= heat_critical_ratio_ ||
      heat_critical_ratio_ > 1.0 || referee_data_timeout_ms_ <= 0 ||
      teammate_data_timeout_ms_ <= 0 || target_lost_timeout_ms_ <= 0 ||
      cover_teammate_hp_threshold_ <= 0 || base_hp_low_ <= 0 || base_hp_low_ > 65535 ||
      outpost_alive_min_hp_ < 0 || outpost_alive_min_hp_ > 65535 ||
      outpost_reference_hp_ <= 0 || outpost_reference_hp_ > 65535 ||
      decision_hold_ms_ < 0 || failure_cooldown_ms_ < 0 ||
      !std::isfinite(action_switch_margin_) || action_switch_margin_ < 0.0F ||
      !std::isfinite(tick_hz_) || tick_hz_ <= 0.0) {
    throw std::invalid_argument("Invalid tactical scorer thresholds, timing, or normalized factors");
  }

  const auto qos = rclcpp::QoS(10);
  game_status_sub_ = create_subscription<pb_rm_interfaces::msg::GameStatus>(
    "referee/game_status", qos,
    [this](const pb_rm_interfaces::msg::GameStatus::SharedPtr msg) {
      game_status_ = *msg;
      last_game_status_stamp_ = now();
    });
  robot_status_sub_ = create_subscription<pb_rm_interfaces::msg::RobotStatus>(
    "referee/robot_status", qos,
      [this](const pb_rm_interfaces::msg::RobotStatus::SharedPtr msg) {
        robot_status_ = *msg;
        last_robot_status_stamp_ = now();
      });
  all_robot_hp_sub_ = create_subscription<pb_rm_interfaces::msg::GameRobotHP>(
    "referee/all_robot_hp", qos,
    [this](const pb_rm_interfaces::msg::GameRobotHP::SharedPtr msg) {
      all_robot_hp_ = *msg;
      last_all_robot_hp_stamp_ = now();
    });
  ground_robot_position_sub_ = create_subscription<pb_rm_interfaces::msg::GroundRobotPosition>(
    "referee/ground_robot_position", qos,
    [this](const pb_rm_interfaces::msg::GroundRobotPosition::SharedPtr msg) {
      ground_robot_position_ = *msg;
      last_ground_robot_position_stamp_ = now();
    });
  event_data_sub_ = create_subscription<pb_rm_interfaces::msg::EventData>(
    "referee/event_data", qos,
    [this](const pb_rm_interfaces::msg::EventData::SharedPtr msg) {
      const bool continuous = event_data_ &&
        isFresh(last_event_data_stamp_, referee_data_timeout_ms_);
      const bool threat = (msg->base_hp > 0 && msg->base_hp < 1.1 * base_hp_low_) ||
        msg->fortress_gain_zone == msg->OCCUPIED_ENEMY ||
        msg->fortress_gain_zone == msg->OCCUPIED_BOTH;
      defense_threat_observations_ = threat ?
        std::min(continuous ? defense_threat_observations_ + 1 : 1, 3) : 0;
      event_data_ = *msg;
      last_event_data_stamp_ = now();
    });
  rfid_status_sub_ = create_subscription<pb_rm_interfaces::msg::RfidStatus>(
    "referee/rfid_status", qos,
    [this](const pb_rm_interfaces::msg::RfidStatus::SharedPtr msg) { rfid_status_ = *msg; });
  tracker_target_sub_ = create_subscription<auto_aim_interfaces::msg::Target>(
    "tracker/target", rclcpp::SensorDataQoS(),
    [this](const auto_aim_interfaces::msg::Target::SharedPtr msg) {
      tracker_target_ = *msg;
      last_target_stamp_ = now();
    });
  simulation_state_sub_ = create_subscription<std_msgs::msg::String>(
    "/referee/simulation_state", qos,
    [this](const std_msgs::msg::String::SharedPtr msg) { onSimulationState(msg); });

  decision_pub_ = create_publisher<TacticalDecision>(decision_topic_, rclcpp::QoS(10));

  tick_hz_ = std::max(tick_hz_, 0.1);
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / tick_hz_));
  timer_ = create_wall_timer(period, std::bind(&TacticalScorerNode::onTimer, this));

  RCLCPP_INFO(
    get_logger(),
    "Tactical scorer started: topic=%s team=%s tick_hz=%.2f hp_low=%d ammo_low=%d base_hp_low=%d",
    decision_topic_.c_str(), team_color_.c_str(), tick_hz_, hp_low_, ammo_low_,
    base_hp_low_);
}

void TacticalScorerNode::onSimulationState(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string key = "world_state_id=";
  const auto pos = msg->data.find(key);
  if (pos != std::string::npos) {
    try {
      const auto begin = pos + key.size();
      const auto end = msg->data.find_first_not_of("0123456789", begin);
      world_state_id_ = static_cast<uint32_t>(std::stoul(msg->data.substr(begin, end - begin)));
    }
    catch (const std::exception &) { }
  }
}

void TacticalScorerNode::onTimer()
{
  const auto draft = score();
  const auto msg = makeDecisionMsg(draft);
  decision_pub_->publish(msg);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "[TacticalScorer] decision_id=%u action=%s score=%.2f reasons=%zu rejected=%zu",
    msg.decision_id, msg.action_name.c_str(), msg.score, msg.top_reasons.size(),
    msg.rejected_actions.size());
}

TacticalScorerNode::DecisionDraft TacticalScorerNode::score()
{
  struct Candidate
  {
    uint8_t action;
    float score{0.0F};
    bool available{false};
    std::vector<std::pair<std::string, double>> factors;
    std::string reason;
    std::string rejection;
    std::string target_id;
  };

  const auto now_time = now();
  std::string safety_gate;
  if (mapping_bootstrap_enabled_ && !mapping_finished_) {
    safety_gate = "mapping_not_finished";
  } else if (!game_status_) {
    safety_gate = "missing_game_status";
  } else if (!isFresh(last_game_status_stamp_, referee_data_timeout_ms_)) {
    safety_gate = "stale_game_status";
  } else if (game_status_->game_progress != game_status_->RUNNING) {
    safety_gate = "game_not_running";
  } else if (!robot_status_) {
    safety_gate = "missing_robot_status";
  } else if (!isFresh(last_robot_status_stamp_, referee_data_timeout_ms_)) {
    safety_gate = "stale_robot_status";
  } else if (!event_data_) {
    safety_gate = "missing_event_data";
  } else if (!isFresh(last_event_data_stamp_, referee_data_timeout_ms_)) {
    // Base/fortress safety cannot be established from an expired event message.
    safety_gate = "stale_event_data";
  }

  const bool resource_low = isRobotResourceLow();
  const bool resource_critical = isRobotResourceCritical();
  const bool base_danger = isBaseDanger();
  const bool target_fresh = isTargetFresh();
  const bool hp_fresh = all_robot_hp_ &&
    isFresh(last_all_robot_hp_stamp_, teammate_data_timeout_ms_);
  const bool positions_fresh = ground_robot_position_ &&
    isFresh(last_ground_robot_position_stamp_, teammate_data_timeout_ms_);
  const double cover_urgency = coverUrgency();
  const bool outpost_available = isOutpostAlive();
  const bool capture_available = isCapturePointInteresting();

  double hp_ready = 0.0;
  double resource_ready = 0.0;
  if (robot_status_) {
    hp_ready = linear(robot_status_->current_hp, hp_critical_, hp_low_);
    const double ammo_ready = linear(
      robot_status_->projectile_allowance_17mm, ammo_critical_, ammo_low_);
    // A zero heat limit carries no usable ratio; preserve operation on existing
    // referee sources that do not publish a heat limit.
    const double heat_ready = robot_status_->shooter_barrel_heat_limit == 0 ? 1.0 :
      1.0 - linear(
      static_cast<double>(robot_status_->shooter_17mm_1_barrel_heat) /
      robot_status_->shooter_barrel_heat_limit, heat_low_ratio_, heat_critical_ratio_);
    resource_ready = std::min({hp_ready, ammo_ready, heat_ready});
  }

  const bool fortress_threat = event_data_ &&
    (event_data_->fortress_gain_zone == event_data_->OCCUPIED_ENEMY ||
    event_data_->fortress_gain_zone == event_data_->OCCUPIED_BOTH);
  const bool ordinary_base_threat = event_data_ && event_data_->base_hp > 0 &&
    event_data_->base_hp < 1.1 * base_hp_low_;
  const bool defense_threat = fortress_threat || ordinary_base_threat;
  if (!event_data_ || !isFresh(last_event_data_stamp_, referee_data_timeout_ms_)) {
    defense_threat_observations_ = 0;
  }

  uint8_t forced_action = 255;
  std::string forced_reason;
  if (!safety_gate.empty()) {
    forced_action = kHoldSafe;
    forced_reason = safety_gate;
  } else if (resource_critical) {
    forced_action = kResupplyHome;
    forced_reason = "resource_critical";
  } else if (base_danger) {
    forced_action = kDefendHome;
    forced_reason = "base_hp_critical";
  }
  const bool safety_cooldown = forced_action != 255 && !available(forced_action);
  const auto candidate = [&](uint8_t action, bool permitted,
      std::vector<std::pair<std::string, double>> factors,
      const std::string & reason, const std::string & rejection = "") {
      Candidate result;
      result.action = action;
      const bool cooled_down = available(action);
      result.available = permitted && cooled_down;
      result.factors = std::move(factors);
      result.reason = reason;
      if (!permitted) {
        result.rejection = rejection;
      } else if (!cooled_down) {
        result.rejection = "failure_cooldown";
      }
      if (!safety_gate.empty() && action != kHoldSafe) {
        result.available = false;
        result.rejection = "hard_gate:" + safety_gate;
      }
      bool zero_factor = false;
      double log_sum = 0.0;
      for (auto & factor : result.factors) {
        factor.second = std::clamp(factor.second, 0.0, 1.0);
        zero_factor = zero_factor || factor.second <= 0.0;
        log_sum += std::log(factor.second > 0.0 ? factor.second : 1.0);
      }
      result.score = result.available ?
        (zero_factor ? 0.0F : static_cast<float>(action_importance_[action] *
        std::exp(log_sum / std::max<std::size_t>(1, result.factors.size())))) : 0.0F;
      return result;
    };

  const double outpost_hp = all_robot_hp_ ? static_cast<double>(
    team_color_ == "red" ? all_robot_hp_->blue_outpost_hp : all_robot_hp_->red_outpost_hp) : 0.0;
  const double outpost_opportunity = outpost_available ?
    std::clamp(0.5 + 0.5 * (1.0 - outpost_hp / static_cast<double>(outpost_reference_hp_)),
    0.0, 1.0) : 0.0;
  const double resource_need = std::clamp(1.0 - resource_ready, 0.0, 1.0);
  const double criticality = resource_low ? std::clamp(std::max({
    1.0 - hp_ready,
    1.0 - (robot_status_ ? linear(
      robot_status_->projectile_allowance_17mm, ammo_critical_, ammo_low_) : 0.0),
    robot_status_ && robot_status_->shooter_barrel_heat_limit > 0 ?
      linear(static_cast<double>(robot_status_->shooter_17mm_1_barrel_heat) /
      robot_status_->shooter_barrel_heat_limit, heat_low_ratio_, heat_critical_ratio_) : 0.0}),
    0.0, 1.0) : 0.0;
  const double threat_level = !defense_threat ? 0.0 :
    (fortress_threat ? 1.0 : std::clamp(
      linear(1.1 * base_hp_low_ - event_data_->base_hp,
      0.0, 0.1 * base_hp_low_), 0.0, 1.0));
  const double threat_persistence = base_danger ? 1.0 : defense_threat_observations_ / 3.0;
  const double match_phase = !game_status_ ? 0.0 :
    (game_status_->stage_remain_time <= 30 ? 1.0 :
    (game_status_->stage_remain_time <= 120 ? 0.85 : 0.70));
  const double capture_state = !capture_available ? 0.0 :
    (event_data_->center_gain_zone == event_data_->OCCUPIED_ENEMY ? 1.0 :
    (event_data_->center_gain_zone == event_data_->OCCUPIED_BOTH ? 0.9 : 0.8));
  std::array<Candidate, 7> candidates = {
    candidate(kHoldSafe, !safety_gate.empty() || safety_cooldown,
      {{"safety", safety_gate.empty() && !safety_cooldown ? 0.0 : 1.0}}, "fallback_safety"),
    candidate(kCoverTeammate, cover_urgency > 0.0,
      {{"teammate_injury", cover_urgency}},
      "low_hp_teammate", !hp_fresh || !positions_fresh ?
      "missing_or_stale_teammate_data" : "no_valid_teammate"),
    candidate(kAttackOutpost, outpost_available,
      {{"outpost_opportunity", outpost_opportunity}, {"resource_ready", resource_ready}},
      "outpost_alive", !hp_fresh ? "missing_or_stale_outpost_data" : "outpost_unavailable"),
    candidate(kResupplyHome, resource_low,
      {{"resource_need", resource_need}, {"criticality", criticality}},
      "robot_resource_low", "resupply_not_needed"),
    candidate(kDefendHome, defense_threat,
      {{"threat_level", threat_level}, {"threat_persistence", threat_persistence}},
      "base_or_fortress_danger", "defense_not_needed"),
    candidate(kCapturePoint, capture_available,
      {{"point_value", capture_state}, {"match_phase", match_phase}},
      "capture_point_interesting", "center_point_unavailable"),
    candidate(kGatherInfo, true,
      {{"information_gap", target_fresh ? 0.25 : 0.90},
        {"target_absence", target_fresh ? 0.0 : 1.0}},
      target_fresh ? "target_already_tracked" : "information_gain_no_target")};
  if (target_fresh) {
    candidates[kAttackOutpost].target_id = tracker_target_->id;
  }

  uint8_t selected = kHoldSafe;
  std::string selection_reason = "utility_max";
  if (forced_action != 255) {
    // A failed emergency action must cool down safely, never expose offensive
    // actions while its hard safety condition still holds.
    selected = safety_cooldown ? kHoldSafe : forced_action;
    selection_reason = safety_cooldown ? "safety_action_failure_cooldown" : "hard_safety_gate";
  } else {
    for (const auto & item : candidates) {
      if (item.available && item.score > candidates[selected].score) {
        selected = item.action;
      }
    }
    if (last_action_ < candidates.size() && last_action_ != selected &&
        last_action_ != kHoldSafe && candidates[last_action_].available) {
      const auto held_ms = (now_time - last_action_change_).seconds() * 1000.0;
      if (held_ms >= 0.0 && held_ms < decision_hold_ms_) {
        selected = last_action_;
        selection_reason = "decision_hold";
      } else if (candidates[selected].score <=
                 candidates[last_action_].score + action_switch_margin_) {
        selected = last_action_;
        selection_reason = "action_switch_margin";
      }
    }
  }

  const auto describe = [&](const Candidate & item) {
      std::ostringstream out;
      out.imbue(std::locale::classic());
      out << std::fixed << std::setprecision(6) << "score=" << item.score
          << ",importance=" << action_importance_[item.action];
      for (const auto & factor : item.factors) {
        out << "," << factor.first << "=" << factor.second;
      }
      return out.str();
    };
  DecisionDraft draft{};
  draft.action = selected;
  draft.action_name = actionName(selected);
  draft.score = candidates[selected].score;
  draft.emergency = forced_action != 255 || selected == kHoldSafe;
  draft.target_id = candidates[selected].target_id;
  draft.top_reasons = {
    forced_action != 255 ? forced_reason : candidates[selected].reason,
    "selection=" + selection_reason,
    actionName(selected) + ":" + describe(candidates[selected])};
  for (const auto & item : candidates) {
    if (item.action == selected) {
      continue;
    }
    const std::string rejection = !item.available ? item.rejection :
      (forced_action != 255 ? "hard_gate:" + forced_reason :
      (selection_reason == "utility_max" ? "lower_utility" : selection_reason));
    draft.rejected_actions.push_back(
      actionName(item.action) + ":" + rejection + ";" + describe(item));
  }
  if (last_action_ != selected) {
    last_action_ = selected;
    last_action_change_ = now_time;
  }
  return draft;
}

TacticalScorerNode::TacticalDecision TacticalScorerNode::makeDecisionMsg(
  const DecisionDraft & draft)
{
  TacticalDecision msg;
  msg.header.stamp = now();
  msg.header.frame_id = "map";
  msg.action = draft.action;
  msg.action_name = actionName(draft.action);
  msg.target_id = draft.target_id;
  msg.score = draft.score;
  msg.top_reasons = draft.top_reasons;
  msg.rejected_actions = draft.rejected_actions;
  msg.emergency = draft.emergency;
  msg.decision_id = ++decision_id_;
  msg.world_state_id = world_state_id_;
  msg.confidence = std::clamp(draft.score, 0.0F, 1.0F);
  msg.valid_for.sec = std::max(0, decision_timeout_ms_) / 1000;
  msg.valid_for.nanosec =
    static_cast<uint32_t>(std::max(0, decision_timeout_ms_) % 1000) * 1000000U;
  msg.preemptible = true;  // The dispatcher enforces priority, not this advisory flag.
  return msg;
}

bool TacticalScorerNode::available(uint8_t action) const
{
  return action < cooldown_until_.size() && now().nanoseconds() >= cooldown_until_[action];
}

bool TacticalScorerNode::isTargetFresh() const
{
  if (!tracker_target_ || !tracker_target_->tracking || tracker_target_->id.empty()) {
    return false;
  }

  return isFresh(last_target_stamp_, target_lost_timeout_ms_);
}

bool TacticalScorerNode::isRobotResourceLow() const
{
  if (!robot_status_) {
    return true;
  }

  const bool hp_low = robot_status_->current_hp < static_cast<uint16_t>(std::max(0, hp_low_));
  const bool ammo_low = robot_status_->projectile_allowance_17mm <
    static_cast<uint16_t>(std::max(0, ammo_low_));
  const bool heat_low = robot_status_->shooter_barrel_heat_limit > 0 &&
    static_cast<double>(robot_status_->shooter_17mm_1_barrel_heat) /
    robot_status_->shooter_barrel_heat_limit >= heat_low_ratio_;
  return hp_low || ammo_low || heat_low;
}

bool TacticalScorerNode::isRobotResourceCritical() const
{
  if (!robot_status_) {
    return true;
  }
  const bool hp_critical = robot_status_->current_hp <= static_cast<uint16_t>(std::max(0, hp_critical_));
  const bool ammo_critical = robot_status_->projectile_allowance_17mm <=
    static_cast<uint16_t>(std::max(0, ammo_critical_));
  const bool heat_critical = robot_status_->shooter_barrel_heat_limit > 0 &&
    static_cast<double>(robot_status_->shooter_17mm_1_barrel_heat) /
    robot_status_->shooter_barrel_heat_limit >= heat_critical_ratio_;
  return hp_critical || ammo_critical || heat_critical;
}

bool TacticalScorerNode::isFresh(const rclcpp::Time & stamp, int timeout_ms) const
{
  const double age_ms = (now() - stamp).seconds() * 1000.0;
  return age_ms >= 0.0 && age_ms <= timeout_ms;
}

bool TacticalScorerNode::isBaseDanger() const
{
  if (!event_data_ || !isFresh(last_event_data_stamp_, referee_data_timeout_ms_)) {
    return false;
  }

  const bool base_low = event_data_->base_hp > 0 &&
                        event_data_->base_hp <= static_cast<uint16_t>(base_hp_low_);
  return base_low;
}

bool TacticalScorerNode::isCapturePointInteresting() const
{
  if (!event_data_ || !isFresh(last_event_data_stamp_, referee_data_timeout_ms_)) {
    return false;
  }

  return event_data_->center_gain_zone == event_data_->UNOCCUPIED ||
         event_data_->center_gain_zone == event_data_->OCCUPIED_ENEMY ||
         event_data_->center_gain_zone == event_data_->OCCUPIED_BOTH;
}

uint16_t TacticalScorerNode::teammateHp(uint8_t robot_id) const
{
  if (!all_robot_hp_) {
    return 0;
  }
  const bool red = team_color_ == "red";
  switch (robot_id) {
    case 1:
      return red ? all_robot_hp_->red_1_robot_hp : all_robot_hp_->blue_1_robot_hp;
    case 2:
      return red ? all_robot_hp_->red_2_robot_hp : all_robot_hp_->blue_2_robot_hp;
    case 3:
      return red ? all_robot_hp_->red_3_robot_hp : all_robot_hp_->blue_3_robot_hp;
    case 4:
      return red ? all_robot_hp_->red_4_robot_hp : all_robot_hp_->blue_4_robot_hp;
    default:
      return 0;
  }
}

double TacticalScorerNode::coverUrgency() const
{
  if (!all_robot_hp_ || !ground_robot_position_ ||
      !isFresh(last_all_robot_hp_stamp_, teammate_data_timeout_ms_) ||
      !isFresh(last_ground_robot_position_stamp_, teammate_data_timeout_ms_)) {
    return 0.0;
  }

  const auto valid_point = [](const geometry_msgs::msg::Point & point) {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           (std::abs(point.x) > 1e-3 || std::abs(point.y) > 1e-3);
  };
  const auto & positions = *ground_robot_position_;
  const std::array<geometry_msgs::msg::Point, 4> points = {
    positions.hero_position, positions.engineer_position,
    positions.standard_3_position, positions.standard_4_position};
  double urgency = 0.0;
  for (uint8_t id = 1; id <= points.size(); ++id) {
    const auto hp = teammateHp(id);
    if (id != sentry_robot_id_ && hp > 0 && hp < cover_teammate_hp_threshold_ &&
        valid_point(points[id - 1])) {
      urgency = std::max(urgency, 1.0 - linear(hp, 0.0, cover_teammate_hp_threshold_));
    }
  }
  return urgency;
}

bool TacticalScorerNode::isOutpostAlive() const
{
  if (!all_robot_hp_ || !isFresh(last_all_robot_hp_stamp_, teammate_data_timeout_ms_)) {
    return false;
  }
  const auto outpost_hp =
    team_color_ == "red" ? all_robot_hp_->blue_outpost_hp : all_robot_hp_->red_outpost_hp;
  return outpost_hp > 0 && outpost_hp >= outpost_alive_min_hp_;
}

std::string TacticalScorerNode::actionName(uint8_t action) const
{
  switch (action) {
    case TacticalDecision::HOLD_SAFE:
      return "HOLD_SAFE";
    case TacticalDecision::COVER_TEAMMATE:
      return "COVER_TEAMMATE";
    case TacticalDecision::ATTACK_OUTPOST:
      return "ATTACK_OUTPOST";
    case TacticalDecision::RESUPPLY_HOME:
      return "RESUPPLY_HOME";
    case TacticalDecision::DEFEND_HOME:
      return "DEFEND_HOME";
    case TacticalDecision::CAPTURE_POINT:
      return "CAPTURE_POINT";
    case TacticalDecision::GATHER_INFO:
      return "GATHER_INFO";
    default:
      return "UNKNOWN";
  }
}

}  // namespace pb2025_sentry_behavior


#ifndef TACTICAL_SCORER_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pb2025_sentry_behavior::TacticalScorerNode>());
  rclcpp::shutdown();
  return 0;
}

#endif

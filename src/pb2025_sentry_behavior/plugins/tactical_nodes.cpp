#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/point.hpp"
#include "pb2025_sentry_decision_interfaces/msg/tactical_decision.hpp"
#include "pb_rm_interfaces/msg/event_data.hpp"
#include "pb_rm_interfaces/msg/game_robot_hp.hpp"
#include "pb_rm_interfaces/msg/ground_robot_position.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{
using TacticalDecision = pb2025_sentry_decision_interfaces::msg::TacticalDecision;

bool getInt(BT::Blackboard::Ptr blackboard, const std::string & key, int & value)
{
  try {
    value = blackboard->get<int>("@" + key);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool getDouble(BT::Blackboard::Ptr blackboard, const std::string & key, double & value)
{
  try {
    value = blackboard->get<double>("@" + key);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool getString(BT::Blackboard::Ptr blackboard, const std::string & key, std::string & value)
{
  try {
    value = blackboard->get<std::string>("@" + key);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool freshBlackboardData(
  BT::Blackboard::Ptr blackboard, const std::string & first_key, const std::string & second_key,
  int timeout_ms)
{
  try {
    const auto first_stamp = blackboard->get<int64_t>("@" + first_key);
    const auto second_stamp = blackboard->get<int64_t>("@" + second_key);
    const auto node = blackboard->get<rclcpp::Node::SharedPtr>("@node");
    const auto now_ns = node->now().nanoseconds();
    const auto first_age = now_ns - first_stamp;
    const auto second_age = now_ns - second_stamp;
    return first_stamp > 0 && second_stamp > 0 && first_age >= 0 && second_age >= 0 &&
           first_age <= static_cast<int64_t>(timeout_ms) * 1000000 &&
           second_age <= static_cast<int64_t>(timeout_ms) * 1000000;
  } catch (const std::exception &) {
    return false;
  }
}

bool validPoint(const geometry_msgs::msg::Point & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         (std::abs(point.x) > 1e-3 || std::abs(point.y) > 1e-3);
}

std::string teamColor(BT::Blackboard::Ptr blackboard)
{
  std::string color = "blue";
  getString(blackboard, "team_color", color);
  return color;
}

uint16_t robotHp(
  const pb_rm_interfaces::msg::GameRobotHP & hp, const std::string & color, uint8_t robot_id)
{
  const bool red = color == "red";
  switch (robot_id) {
    case 1:
      return red ? hp.red_1_robot_hp : hp.blue_1_robot_hp;
    case 2:
      return red ? hp.red_2_robot_hp : hp.blue_2_robot_hp;
    case 3:
      return red ? hp.red_3_robot_hp : hp.blue_3_robot_hp;
    case 4:
      return red ? hp.red_4_robot_hp : hp.blue_4_robot_hp;
    default:
      return 0;
  }
}

const geometry_msgs::msg::Point & robotPosition(
  const pb_rm_interfaces::msg::GroundRobotPosition & positions, uint8_t robot_id)
{
  switch (robot_id) {
    case 1:
      return positions.hero_position;
    case 2:
      return positions.engineer_position;
    case 3:
      return positions.standard_3_position;
    default:
      return positions.standard_4_position;
  }
}

class IsTacticalDecisionFreshCondition : public BT::SimpleConditionNode
{
public:
  IsTacticalDecisionFreshCondition(const std::string & name, const BT::NodeConfig & config)
  : BT::SimpleConditionNode(
      name, std::bind(&IsTacticalDecisionFreshCondition::check, this), config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<TacticalDecision>("decision", "{@tactical_decision}"),
      BT::InputPort<int64_t>("decision_stamp_ns", "{@tactical_decision_stamp_ns}"),
      BT::InputPort<int>("timeout_ms", 1000, "Decision freshness timeout in milliseconds")};
  }

private:
  BT::NodeStatus check()
  {
    auto decision = getInput<TacticalDecision>("decision");
    auto stamp = getInput<int64_t>("decision_stamp_ns");
    int timeout_ms = 1000;
    getInput("timeout_ms", timeout_ms);
    if (!decision || !stamp || *stamp <= 0) {
      return BT::NodeStatus::FAILURE;
    }
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    (void)now;
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("@node");
    const auto age_ms = (node->now().nanoseconds() - *stamp) / 1000000;
    return age_ms >= 0 && age_ms <= timeout_ms ? BT::NodeStatus::SUCCESS
                                                : BT::NodeStatus::FAILURE;
  }
};

class IsTacticalActionCondition : public BT::SimpleConditionNode
{
public:
  IsTacticalActionCondition(const std::string & name, const BT::NodeConfig & config)
  : BT::SimpleConditionNode(name, std::bind(&IsTacticalActionCondition::check, this), config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("action", 0, "Expected tactical action enum"),
      BT::InputPort<TacticalDecision>("decision", "{@tactical_decision}"),
      BT::InputPort<bool>("allow_stale", false, "Allow stale tactical decisions"),
      BT::InputPort<int>("decision_timeout_ms", 1000, "Decision freshness timeout in milliseconds")};
  }

private:
  BT::NodeStatus check()
  {
    auto decision = getInput<TacticalDecision>("decision");
    if (!decision) {
      return BT::NodeStatus::FAILURE;
    }
    int expected = 0;
    getInput("action", expected);
    bool allow_stale = false;
    getInput("allow_stale", allow_stale);
    if (!allow_stale) {
      auto stamp = config().blackboard->get<int64_t>("@tactical_decision_stamp_ns");
      int timeout_ms = 1000;
      getInput("decision_timeout_ms", timeout_ms);
      auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("@node");
      const auto age_ms = (node->now().nanoseconds() - stamp) / 1000000;
      if (stamp <= 0 || age_ms < 0 || age_ms > timeout_ms) {
        return BT::NodeStatus::FAILURE;
      }
    }

    int active = -1;
    getInt(config().blackboard, "active_tactical_action", active);
    const bool emergency =
      decision->emergency || expected == TacticalDecision::HOLD_SAFE ||
      expected == TacticalDecision::RESUPPLY_HOME || expected == TacticalDecision::DEFEND_HOME;

    if (active >= 0 && active == expected) {
      if (decision->emergency && decision->action != static_cast<uint8_t>(expected)) {
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::SUCCESS;
    }
    if (active < 0 && decision->action == static_cast<uint8_t>(expected)) {
      config().blackboard->set("@active_tactical_action", expected);
      return BT::NodeStatus::SUCCESS;
    }
    if (emergency && decision->action == static_cast<uint8_t>(expected)) {
      config().blackboard->set("@active_tactical_action", expected);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};

class IsOutpostAliveCondition : public BT::SimpleConditionNode
{
public:
  IsOutpostAliveCondition(const std::string & name, const BT::NodeConfig & config)
  : BT::SimpleConditionNode(name, std::bind(&IsOutpostAliveCondition::check, this), config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<pb_rm_interfaces::msg::GameRobotHP>(
        "hp", "{@referee_allRobotHP}"),
      BT::InputPort<std::string>("team_color", "blue"),
      BT::InputPort<int>("min_hp", 1, "Minimum outpost HP")};
  }

private:
  BT::NodeStatus check()
  {
    auto hp = getInput<pb_rm_interfaces::msg::GameRobotHP>("hp");
    std::string color = "blue";
    getInput("team_color", color);
    int min_hp = 1;
    getInput("min_hp", min_hp);
    if (!hp) {
      return BT::NodeStatus::FAILURE;
    }
    const auto outpost_hp = color == "red" ? hp->blue_outpost_hp : hp->red_outpost_hp;
    return outpost_hp >= static_cast<uint16_t>(std::max(0, min_hp)) ? BT::NodeStatus::SUCCESS
                                                                     : BT::NodeStatus::FAILURE;
  }
};

class IsTeammateCoverCandidateCondition : public BT::SimpleConditionNode
{
public:
  IsTeammateCoverCandidateCondition(
    const std::string & name, const BT::NodeConfig & config)
  : BT::SimpleConditionNode(
      name, std::bind(&IsTeammateCoverCandidateCondition::check, this), config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<pb_rm_interfaces::msg::GameRobotHP>("hp", "{@referee_allRobotHP}"),
      BT::InputPort<pb_rm_interfaces::msg::GroundRobotPosition>(
        "positions", "{@referee_groundRobotPosition}"),
      BT::InputPort<std::string>("team_color", "blue"),
      BT::InputPort<int>("sentry_robot_id", 7, "Own sentry robot ID"),
      BT::InputPort<int>("hp_threshold", 250, "Teammate HP threshold"),
      BT::InputPort<int>("data_timeout_ms", 500, "Maximum referee data age")};
  }

private:
  BT::NodeStatus check()
  {
    auto hp = getInput<pb_rm_interfaces::msg::GameRobotHP>("hp");
    auto positions = getInput<pb_rm_interfaces::msg::GroundRobotPosition>("positions");
    if (!hp || !positions) {
      return BT::NodeStatus::FAILURE;
    }
    int timeout_ms = 500;
    getInput("data_timeout_ms", timeout_ms);
    if (!freshBlackboardData(
          config().blackboard, "referee_allRobotHP_stamp_ns",
          "referee_groundRobotPosition_stamp_ns", timeout_ms)) {
      return BT::NodeStatus::FAILURE;
    }
    std::string color = "blue";
    int sentry_id = 7;
    int threshold = 250;
    getInput("team_color", color);
    getInput("sentry_robot_id", sentry_id);
    getInput("hp_threshold", threshold);
    for (uint8_t id : {uint8_t{1}, uint8_t{2}, uint8_t{3}, uint8_t{4}}) {
      if (id == sentry_id) {
        continue;
      }
      const auto & point = robotPosition(*positions, id);
      if (robotHp(*hp, color, id) > 0 &&
          robotHp(*hp, color, id) < static_cast<uint16_t>(std::max(0, threshold)) &&
          validPoint(point)) {
        return BT::NodeStatus::SUCCESS;
      }
    }
    return BT::NodeStatus::FAILURE;
  }
};

class CalculateTeammateCoverGoalAction : public BT::SyncActionNode
{
public:
  CalculateTeammateCoverGoalAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<pb_rm_interfaces::msg::GameRobotHP>("hp", "{@referee_allRobotHP}"),
      BT::InputPort<pb_rm_interfaces::msg::GroundRobotPosition>(
        "positions", "{@referee_groundRobotPosition}"),
      BT::InputPort<std::string>("team_color", "blue"),
      BT::InputPort<int>("sentry_robot_id", 7, "Own sentry robot ID"),
      BT::InputPort<int>("hp_threshold", 250, "Teammate HP threshold"),
      BT::InputPort<int>("data_timeout_ms", 500, "Maximum referee data age"),
      BT::InputPort<double>("home_reference_x", 0.0, "Retreat reference X"),
      BT::InputPort<double>("home_reference_y", 0.0, "Retreat reference Y"),
      BT::InputPort<double>("rear_distance", 1.5, "Distance behind teammate"),
      BT::InputPort<double>("side_offset", 1.0, "Lateral cover offset"),
      BT::InputPort<double>("side_sign", 1.0, "Lateral cover side sign"),
      BT::OutputPort<std::string>("goal", "{cover_goal}"),
      BT::OutputPort<std::string>("target_id", "{cover_target_id}")};
  }

  BT::NodeStatus tick() override
  {
    auto hp = getInput<pb_rm_interfaces::msg::GameRobotHP>("hp");
    auto positions = getInput<pb_rm_interfaces::msg::GroundRobotPosition>("positions");
    if (!hp || !positions) {
      return BT::NodeStatus::FAILURE;
    }
    int timeout_ms = 500;
    getInput("data_timeout_ms", timeout_ms);
    if (!freshBlackboardData(
          config().blackboard, "referee_allRobotHP_stamp_ns",
          "referee_groundRobotPosition_stamp_ns", timeout_ms)) {
      return BT::NodeStatus::FAILURE;
    }
    std::string color = "blue";
    int sentry_id = 7;
    int threshold = 250;
    double home_x = 0.0;
    double home_y = 0.0;
    double rear = 1.5;
    double side = 1.0;
    double side_sign = 1.0;
    getInput("team_color", color);
    getInput("sentry_robot_id", sentry_id);
    getInput("hp_threshold", threshold);
    getInput("home_reference_x", home_x);
    getInput("home_reference_y", home_y);
    getInput("rear_distance", rear);
    getInput("side_offset", side);
    getInput("side_sign", side_sign);

    uint8_t selected_id = 0;
    uint16_t selected_hp = std::numeric_limits<uint16_t>::max();
    for (uint8_t id : {uint8_t{1}, uint8_t{2}, uint8_t{3}, uint8_t{4}}) {
      const auto current_hp = robotHp(*hp, color, id);
      if (id != sentry_id && current_hp > 0 &&
          current_hp < static_cast<uint16_t>(std::max(0, threshold)) &&
          validPoint(robotPosition(*positions, id)) && current_hp < selected_hp) {
        selected_id = id;
        selected_hp = current_hp;
      }
    }
    if (selected_id == 0) {
      return BT::NodeStatus::FAILURE;
    }

    const auto & teammate = robotPosition(*positions, selected_id);
    double dx = home_x - teammate.x;
    double dy = home_y - teammate.y;
    const double length = std::hypot(dx, dy);
    if (length < 1e-6) {
      return BT::NodeStatus::FAILURE;
    }
    dx /= length;
    dy /= length;
    const double side_x = -dy * side_sign;
    const double side_y = dx * side_sign;
    const double cover_x = teammate.x + dx * rear + side_x * side;
    const double cover_y = teammate.y + dy * rear + side_y * side;
    const double yaw = std::atan2(teammate.y - cover_y, teammate.x - cover_x);
    setOutput("goal", std::to_string(cover_x) + ";" + std::to_string(cover_y) + ";" +
                         std::to_string(yaw));
    setOutput("target_id", std::to_string(selected_id));
    return BT::NodeStatus::SUCCESS;
  }
};

class SelectDefenseGoalAction : public BT::SyncActionNode
{
public:
  SelectDefenseGoalAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<pb_rm_interfaces::msg::EventData>("event", "{@referee_eventData}"),
      BT::InputPort<int>("base_hp_threshold", 5000, "Base HP threshold"),
      BT::InputPort<std::string>("base_goal", "0;0;0"),
      BT::InputPort<std::string>("fortress_goal", "0;0;0"),
      BT::OutputPort<std::string>("goal", "{defense_goal}")};
  }

  BT::NodeStatus tick() override
  {
    auto event = getInput<pb_rm_interfaces::msg::EventData>("event");
    if (!event) {
      return BT::NodeStatus::FAILURE;
    }
    int threshold = 5000;
    std::string base_goal = "0;0;0";
    std::string fortress_goal = "0;0;0";
    getInput("base_hp_threshold", threshold);
    getInput("base_goal", base_goal);
    getInput("fortress_goal", fortress_goal);
    const bool fortress_danger =
      event->fortress_gain_zone == event->OCCUPIED_ENEMY ||
      event->fortress_gain_zone == event->OCCUPIED_BOTH;
    const bool base_danger =
      event->base_hp > 0 && event->base_hp <= static_cast<uint16_t>(std::max(0, threshold));
    if (base_danger) {
      setOutput("goal", base_goal);
      return BT::NodeStatus::SUCCESS;
    }
    if (fortress_danger) {
      setOutput("goal", fortress_goal);
      return BT::NodeStatus::SUCCESS;
    }
    if (event->base_hp > 0 && event->base_hp < 1.1 * threshold) {
      setOutput("goal", base_goal);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};

class ClearTacticalAction : public BT::SyncActionNode
{
public:
  ClearTacticalAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    config().blackboard->set("@active_tactical_action", -1);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace pb2025_sentry_behavior

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsTacticalDecisionFreshCondition>(
    "IsTacticalDecisionFresh");
  factory.registerNodeType<pb2025_sentry_behavior::IsTacticalActionCondition>(
    "IsTacticalAction");
  factory.registerNodeType<pb2025_sentry_behavior::IsOutpostAliveCondition>("IsOutpostAlive");
  factory.registerNodeType<pb2025_sentry_behavior::IsTeammateCoverCandidateCondition>(
    "IsTeammateCoverCandidate");
  factory.registerNodeType<pb2025_sentry_behavior::CalculateTeammateCoverGoalAction>(
    "CalculateTeammateCoverGoal");
  factory.registerNodeType<pb2025_sentry_behavior::SelectDefenseGoalAction>(
    "SelectDefenseGoal");
  factory.registerNodeType<pb2025_sentry_behavior::ClearTacticalAction>("ClearTacticalAction");
}

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

#include <array>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/control_node.h"
#include "geometry_msgs/msg/twist.hpp"
#include "pb2025_sentry_behavior/tactical_navigation.hpp"
#include "pb2025_sentry_decision_interfaces/msg/tactical_decision.hpp"
#include "pb2025_sentry_decision_interfaces/msg/tactical_execution.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/game_robot_hp.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "std_msgs/msg/bool.hpp"

namespace pb2025_sentry_behavior
{
using Decision = pb2025_sentry_decision_interfaces::msg::TacticalDecision;
using Execution = pb2025_sentry_decision_interfaces::msg::TacticalExecution;

template<typename T>
T read(const BT::NodeConfig & config, const std::string & key, T fallback = T{})
{
  T value;
  return config.blackboard->get("@" + key, value) ? value : fallback;
}

class TacticalDispatch : public BT::ControlNode
{
public:
  TacticalDispatch(const std::string & name, const BT::NodeConfig & config)
  : BT::ControlNode(name, config), node_(read<rclcpp::Node::SharedPtr>(config, "node")),
    navigation_(read<std::shared_ptr<TacticalNavigation>>(config, "tactical_navigation"))
  {
    if (!navigation_) { navigation_ = std::make_shared<TacticalNavigation>(node_); }
    config.blackboard->set("@tactical_navigation", navigation_);
    events_ = node_->create_publisher<Execution>("/tactical_execution", 10);
    velocity_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    if (childrenCount() != 7) { throw BT::RuntimeError("TacticalDispatch requires actions 0..6"); }
    navigation_->poll();
    const auto now = node_->now().nanoseconds();
    if (pending_) {
      stop();
      if (navigation_->busy()) {
        if (navigation_->cancelUnconfirmed(
              read<int>(config(), "navigation_cancel_timeout_ms", 2000))) {
          emit("RUNNING", "navigation_cancel_unconfirmed");
        }
        return BT::NodeStatus::RUNNING;
      }
      finish(pending_state_, pending_reason_);
    }

    // A restarted tree must preserve the previous tree's cancellation barrier.
    if (active_ < 0 && navigation_->busy()) {
      navigation_->cancel();
      stop();
      return BT::NodeStatus::RUNNING;
    }
    const auto game = read<pb_rm_interfaces::msg::GameStatus>(config(), "referee_gameStatus");
    if (game.game_progress != game.RUNNING) {
      if (active_ >= 0) { terminate("PREEMPTED", "game_not_running"); }
      stop();
      return BT::NodeStatus::RUNNING;
    }
    if (read<bool>(config(), "mapping_bootstrap_enabled") &&
        !read<std_msgs::msg::Bool>(config(), "mapping_bootstrap_finished").data) {
      if (active_ >= 0) { terminate("PREEMPTED", "mapping_not_finished"); stop(); }
      return BT::NodeStatus::RUNNING;
    }

    auto decision = read<Decision>(config(), "tactical_decision");
    const auto stamp = read<int64_t>(config(), "tactical_decision_stamp_ns");
    const bool fresh = stamp > 0 && now >= stamp && now - stamp <=
      int64_t(read<int>(config(), "decision_timeout_ms", 1000)) * 1000000;
    const bool valid = fresh && decision.action <= 6;
    int desired = valid ? decision.action : Decision::HOLD_SAFE;
    if (desired != 0 && now < cooldown_[desired]) { desired = Decision::HOLD_SAFE; }
    if (!valid || desired != decision.action) {
      decision = Decision{};
      decision.action = Decision::HOLD_SAFE;
      decision.action_name = "HOLD_SAFE";
      decision.emergency = true;
    }
    if (active_ >= 0 && active_ != desired) {
      terminate("PREEMPTED", valid ? "decision_changed" : "decision_stale");
      if (pending_) { stop(); return BT::NodeStatus::RUNNING; }
    }
    if (active_ < 0) {
      active_ = desired;
      decision_ = decision;
      config().blackboard->set("@active_tactical_action", active_);
      config().blackboard->set("@active_tactical_decision", decision_);
      config().blackboard->set(
        "@tactical_failure_reason", std::string("action_precondition_failed"));
      emit("STARTED", "action_started");
      emit("RUNNING", "action_running");
    }
    const auto status = children_nodes_[active_]->executeTick();
    if (status == BT::NodeStatus::SUCCESS) {
      terminate("SUCCEEDED", "action_completed");
    } else if (status == BT::NodeStatus::FAILURE) {
      terminate("FAILED", read<std::string>(config(), "tactical_failure_reason", "action_failed"));
    }
    return BT::NodeStatus::RUNNING;
  }

  void halt() override
  {
    if (active_ >= 0) {
      haltChild(active_);
      navigation_->cancel();
      emit(navigation_->busy() ? "FAILED" : "PREEMPTED",
           navigation_->busy() ? "tree_halted_cancel_unconfirmed" : "tree_halted");
      active_ = -1;
      pending_ = false;
      config().blackboard->set("@active_tactical_action", -1);
    }
    stop();
    BT::ControlNode::halt();
  }

private:
  void stop() { velocity_->publish(geometry_msgs::msg::Twist{}); }
  void terminate(const std::string & state, const std::string & reason)
  {
    haltChild(active_);
    navigation_->cancel();
    if (state != "SUCCEEDED") { stop(); }
    if (navigation_->busy()) {
      pending_ = true;
      pending_state_ = state;
      pending_reason_ = reason;
      emit("RUNNING", "navigation_cancellation_pending:" + reason);
    } else { finish(state, reason); }
  }
  void finish(const std::string & state, const std::string & reason)
  {
    emit(state, reason);
    if (state == "FAILED" && active_ > 0) {
      cooldown_[active_] = node_->now().nanoseconds() +
        int64_t(read<int>(config(), "action_failure_cooldown_ms", 5000)) * 1000000;
    }
    active_ = -1;
    pending_ = false;
    config().blackboard->set("@active_tactical_action", -1);
  }
  void emit(const std::string & state, const std::string & reason)
  {
    Execution event;
    event.header.stamp = node_->now();
    event.action = decision_.action; event.decision_id = decision_.decision_id;
    event.world_state_id = decision_.world_state_id;
    event.state = state; event.reason = reason;
    events_->publish(event);
  }
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<TacticalNavigation> navigation_;
  rclcpp::Publisher<Execution>::SharedPtr events_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_;
  Decision decision_;
  int active_{-1};
  std::array<int64_t, 7> cooldown_{};
  bool pending_{false};
  std::string pending_state_, pending_reason_;
};

class TacticalWait : public BT::StatefulActionNode
{
public:
  TacticalWait(const std::string & name, const BT::NodeConfig & config)
  : BT::StatefulActionNode(name, config), node_(read<rclcpp::Node::SharedPtr>(config, "node"))
  {
    velocity_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("mode", "dwell", "hold, dwell, resupply, outpost, gather"),
      BT::InputPort<int>("duration_ms", 10000, "Wait duration or deadline")};
  }
  BT::NodeStatus onStart() override
  {
    started_ = node_->now().nanoseconds();
    unlocked_since_ = started_;
    return onRunning();
  }
  BT::NodeStatus onRunning() override
  {
    velocity_->publish(geometry_msgs::msg::Twist{});
    const auto now = node_->now().nanoseconds();
    if (now < started_) { started_ = now; unlocked_since_ = now; }
    const auto mode = getInput<std::string>("mode").value();
    const auto duration = int64_t(getInput<int>("duration_ms").value()) * 1000000;
    if (mode == "hold") { return BT::NodeStatus::RUNNING; }
    if (mode == "gather") {
      const auto stamp = read<int64_t>(config(), "auto_aim_target_locked_stamp_ns");
      const auto timeout = int64_t(read<int>(config(), "target_locked_timeout_ms", 1000)) * 1000000;
      const bool locked = read<std_msgs::msg::Bool>(config(), "auto_aim_target_locked").data;
      if (locked && stamp > 0 && now >= stamp && now - stamp <= timeout) {
        unlocked_since_ = -1;
      } else if (unlocked_since_ < 0) {
        // Start at the actual false-message/staleness boundary, not a later BT tick.
        unlocked_since_ = std::max(started_, locked ? stamp + timeout : stamp);
      }
      return unlocked_since_ >= 0 && now - unlocked_since_ >= duration ?
        BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
    }
    if (mode == "resupply") {
      const auto hp = read<pb_rm_interfaces::msg::RobotStatus>(config(), "referee_robotStatus");
      if (fresh("referee_robotStatus", now) &&
          hp.current_hp >= read<int>(config(), "resupply_hp_target", 400) &&
          hp.projectile_allowance_17mm >= read<int>(config(), "resupply_ammo_target", 150)) {
        return BT::NodeStatus::SUCCESS;
      }
    } else if (mode == "outpost") {
      const auto hp = read<pb_rm_interfaces::msg::GameRobotHP>(config(), "referee_allRobotHP");
      const bool red = read<std::string>(config(), "team_color", "blue") == "red";
      if (fresh("referee_allRobotHP", now) &&
          (red ? hp.blue_outpost_hp : hp.red_outpost_hp) == 0) {
        return BT::NodeStatus::SUCCESS;
      }
    } else if (mode != "dwell") {
      config().blackboard->set("@tactical_failure_reason", std::string("invalid_wait_mode"));
      return BT::NodeStatus::FAILURE;
    }
    if (now - started_ < duration) { return BT::NodeStatus::RUNNING; }
    if (mode == "dwell") { return BT::NodeStatus::SUCCESS; }
    config().blackboard->set("@tactical_failure_reason", mode + "_timeout");
    return BT::NodeStatus::FAILURE;
  }
  void onHalted() override { velocity_->publish(geometry_msgs::msg::Twist{}); }
private:
  bool fresh(const std::string & key, int64_t now) const
  {
    const auto stamp = read<int64_t>(config(), key + "_stamp_ns");
    return config().blackboard->getEntry("@" + key) != nullptr &&
      stamp > 0 && now >= stamp && now - stamp <=
      int64_t(read<int>(config(), "referee_data_timeout_ms", 1000)) * 1000000;
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_;
  int64_t started_{0}, unlocked_since_{0};
};
}  // namespace pb2025_sentry_behavior

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::TacticalDispatch>("TacticalDispatch");
  factory.registerNodeType<pb2025_sentry_behavior::TacticalWait>("TacticalWait");
}

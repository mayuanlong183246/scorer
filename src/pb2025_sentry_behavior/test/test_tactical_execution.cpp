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

#include <gtest/gtest.h>
#include <chrono>
#include <fstream>
#include <thread>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_ros2/bt_utils.hpp"
#include "pb2025_sentry_behavior/tactical_navigation.hpp"
#include "pb2025_sentry_decision_interfaces/msg/tactical_decision.hpp"
#include "pb2025_sentry_decision_interfaces/msg/tactical_execution.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/game_robot_hp.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "pb_rm_interfaces/msg/ground_robot_position.hpp"
#include "pb_rm_interfaces/msg/event_data.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using Decision = pb2025_sentry_decision_interfaces::msg::TacticalDecision;
using Execution = pb2025_sentry_decision_interfaces::msg::TacticalExecution;
using Nav = pb2025_sentry_behavior::TacticalNavigation;

class TacticalTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { if (!rclcpp::ok()) { rclcpp::init(0, nullptr); } }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("tactical_test");
    bb = BT::Blackboard::create();
    bb->set("node", node);
    set("mapping_bootstrap_enabled", false);
    set("mapping_bootstrap_finished", std_msgs::msg::Bool{});
    set("team_color", std::string("blue"));
    for (const auto & key : {"outpost_attack_goal", "resupply_goal", "defend_base_goal",
         "defend_fortress_goal", "capture_point_a_goal", "capture_point_b_goal",
         "gather_info_goal"}) {
      set(key, std::string("1;2;0"));
    }
    set("capture_point_b_goal", std::string("3;4;0"));
    for (const auto & key : {"cover_dwell_ms", "defend_dwell_ms", "capture_point_a_dwell_ms",
         "capture_point_b_dwell_ms", "gather_info_dwell_ms"}) { set(key, 120); }
    set("outpost_attack_timeout_ms", 250);
    set("resupply_timeout_ms", 250);
    set("navigation_timeout_ms", 1000);
    set("navigation_cancel_timeout_ms", 60);
    set("action_failure_cooldown_ms", 200);
    set("decision_timeout_ms", 1000);
    set("target_locked_timeout_ms", 60);
    set("referee_data_timeout_ms", 1000);
    set("resupply_hp_target", 400); set("resupply_ammo_target", 150);
    set("base_hp_low", 5000); set("outpost_alive_min_hp", 1);
    set("sentry_robot_id", 7); set("cover_teammate_hp_threshold", 250);
    set("teammate_data_timeout_ms", 1000);
    set("cover_home_reference_x", 0.0); set("cover_home_reference_y", 0.0);
    set("cover_rear_distance", 1.5); set("cover_side_offset", 1.0); set("cover_side_sign", 1.0);
    pb_rm_interfaces::msg::GameStatus game; game.game_progress = 4; game.stage_remain_time = 400;
    set("referee_gameStatus", game);
    hp.red_outpost_hp = 100; hp.blue_outpost_hp = 0; hp.blue_3_robot_hp = 100;
    set("referee_allRobotHP", hp);
    pb_rm_interfaces::msg::GroundRobotPosition positions; positions.standard_3_position.x = 5;
    set("referee_groundRobotPosition", positions);
    pb_rm_interfaces::msg::EventData event; event.base_hp = 4000;
    set("referee_eventData", event);
    robot.current_hp = 400; robot.projectile_allowance_17mm = 150;
    set("referee_robotStatus", robot);
    auto qos = rclcpp::QoS(10);
    events_sub = node->create_subscription<Execution>("/tactical_execution", qos,
      [this](Execution::SharedPtr msg) { events.push_back(*msg); });
    stop_sub = node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", qos,
      [this](geometry_msgs::msg::Twist::SharedPtr) { ++stops; });
    server = rclcpp_action::create_server<Nav::Action>(node, "navigate_to_pose",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Nav::Action::Goal> goal) {
        requested.push_back(*goal);
        return reject ? rclcpp_action::GoalResponse::REJECT :
          rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav::Action>>) {
        ++cancels;
        return reject_cancel ? rclcpp_action::CancelResponse::REJECT :
          rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav::Action>> handle) {
        handles.push_back(handle);
      });
    BT::RosNodeParams params; params.nh = node;
    for (const auto & lib : {"tactical_nodes", "tactical_execution", "tactical_navigate_to_pose"}) {
      BT::LoadPlugin(factory, std::string(PLUGIN_DIR) + "/lib" + lib + ".so", params);
    }
    factory.registerBehaviorTreeFromFile(
      std::string(ROOT_DIR) + "behavior_trees/sentry_tactical.xml");
    tree = factory.createTree("sentry_tactical", bb);
    choose(6);
    auto transport = bb->get<std::shared_ptr<Nav>>("tactical_navigation");
    // ROS graph discovery happens before sending the first goal.
    for (int i = 0; i < 20; ++i) { rclcpp::spin_some(node); std::this_thread::sleep_for(5ms); }
  }
  void TearDown() override
  {
    tree.haltTree();
    auto transport = bb->get<std::shared_ptr<Nav>>("tactical_navigation");
    for (int i = 0; i < 15; ++i) {
      rclcpp::spin_some(node);
      for (auto & h : handles) {
        if (h->is_canceling()) { h->canceled(std::make_shared<Nav::Action::Result>()); }
      }
      transport->poll();
      std::this_thread::sleep_for(2ms);
    }
  }
  template<typename T> void set(const std::string & key, const T & value)
  {
    bb->set(key, value); bb->set(key + "_stamp_ns", node->now().nanoseconds());
  }
  void choose(int action)
  {
    decision.action = action; ++decision.decision_id; decision.world_state_id = 42;
    set("tactical_decision", decision);
  }
  void step(bool refresh = true)
  {
    if (refresh) { set("tactical_decision", decision); }
    rclcpp::spin_some(node);
    for (auto & h : handles) {
      if (h->is_canceling() && complete_cancel) {
        h->canceled(std::make_shared<Nav::Action::Result>());
      }
    }
    tree.tickOnce(); rclcpp::spin_some(node); std::this_thread::sleep_for(3ms);
  }
  template<typename Predicate> bool until(Predicate predicate, int milliseconds = 700)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(milliseconds);
    do { step(); if (predicate()) { return true; } } while (std::chrono::steady_clock::now() < end);
    return false;
  }
  void run(int milliseconds, bool refresh = true)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(milliseconds);
    while (std::chrono::steady_clock::now() < end) { step(refresh); }
  }
  bool has(int action, const std::string & state) const
  {
    for (const auto & e : events) { if (e.action == action && e.state == state) { return true; } }
    return false;
  }
  void arrive(size_t index = 0)
  {
    ASSERT_TRUE(until([&] { return handles.size() > index; }));
    handles[index]->succeed(std::make_shared<Nav::Action::Result>());
    step();
  }
  rclcpp::Node::SharedPtr node;
  BT::Blackboard::Ptr bb;
  BT::BehaviorTreeFactory factory;
  BT::Tree tree;
  Decision decision;
  pb_rm_interfaces::msg::GameRobotHP hp;
  pb_rm_interfaces::msg::RobotStatus robot;
  rclcpp_action::Server<Nav::Action>::SharedPtr server;
  std::vector<std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav::Action>>> handles;
  std::vector<Nav::Action::Goal> requested;
  std::vector<Execution> events;
  rclcpp::Subscription<Execution>::SharedPtr events_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr stop_sub;
  bool reject{false}, reject_cancel{false}, complete_cancel{true};
  int cancels{0}, stops{0};
};

TEST_F(TacticalTest, GatesDoNotInterfereWithMapping)
{
  set("mapping_bootstrap_enabled", true); run(80);
  EXPECT_TRUE(requested.empty()); EXPECT_EQ(stops, 0); EXPECT_TRUE(events.empty());
  std_msgs::msg::Bool done; done.data = true; set("mapping_bootstrap_finished", done);
  ASSERT_TRUE(until([&] { return handles.size() == 1; }));
  pb_rm_interfaces::msg::GameStatus game; game.game_progress = 5; set("referee_gameStatus", game);
  ASSERT_TRUE(until([&] { return has(6, "PREEMPTED"); }));
  EXPECT_GT(stops, 0); EXPECT_EQ(requested.size(), 1U);
}

TEST_F(TacticalTest, NotStartedDoesNotNavigate)
{
  pb_rm_interfaces::msg::GameStatus game; game.game_progress = 1; set("referee_gameStatus", game);
  run(60); EXPECT_TRUE(requested.empty()); EXPECT_GT(stops, 0);
}

TEST_F(TacticalTest, CaptureCanBePreemptedByLatestOrdinaryDecision)
{
  choose(5); const auto id = decision.decision_id; arrive();
  choose(1);
  ASSERT_TRUE(until([&] { return handles.size() == 2; }));
  EXPECT_EQ(bb->get<int>("active_tactical_action"), 1);
  EXPECT_EQ(requested[1].pose.pose.position.x, 3.5);
  EXPECT_TRUE(has(5, "PREEMPTED"));
  handles[1]->succeed(std::make_shared<Nav::Action::Result>());
  ASSERT_TRUE(until([&] { return has(1, "SUCCEEDED"); }));
  for (const auto & e : events) { if (e.action == 5) { EXPECT_EQ(e.decision_id, id); } }
}

TEST_F(TacticalTest, ResupplyThresholdsIgnoreHeat)
{
  choose(3); robot.current_hp = 399; set("referee_robotStatus", robot); arrive(); run(35);
  EXPECT_FALSE(has(3, "SUCCEEDED"));
  robot.current_hp = 400; robot.projectile_allowance_17mm = 149;
  set("referee_robotStatus", robot); run(35); EXPECT_FALSE(has(3, "SUCCEEDED"));
  robot.projectile_allowance_17mm = 150; robot.shooter_17mm_1_barrel_heat = 400;
  set("referee_robotStatus", robot);
  ASSERT_TRUE(until([&] { return has(3, "SUCCEEDED"); }));
}

TEST_F(TacticalTest, ResupplyTimeoutAndCooldown)
{
  choose(3); robot.current_hp = 100; set("referee_robotStatus", robot); arrive();
  ASSERT_TRUE(until([&] { return has(3, "FAILED"); }));
  run(70); EXPECT_EQ(requested.size(), 1U);
  ASSERT_TRUE(until([&] { return requested.size() == 2; }));
}

TEST_F(TacticalTest, OutpostUsesEnemyAndNeedsReceivedFreshHp)
{
  choose(2); arrive(); run(30); EXPECT_FALSE(has(2, "SUCCEEDED"));
  bb->unset("referee_allRobotHP"); run(30); EXPECT_FALSE(has(2, "SUCCEEDED"));
  hp.red_outpost_hp = 0; hp.blue_outpost_hp = 1000; set("referee_allRobotHP", hp);
  ASSERT_TRUE(until([&] { return has(2, "SUCCEEDED"); }));
}

TEST_F(TacticalTest, OutpostTimeout)
{
  choose(2); arrive(); ASSERT_TRUE(until([&] { return has(2, "FAILED"); }));
}

TEST_F(TacticalTest, CoverAndDefenseDwell)
{
  choose(1); arrive(); run(30); EXPECT_FALSE(has(1, "SUCCEEDED"));
  ASSERT_TRUE(until([&] { return has(1, "SUCCEEDED"); }));
  choose(4); arrive(1); run(30); EXPECT_FALSE(has(4, "SUCCEEDED"));
  ASSERT_TRUE(until([&] { return has(4, "SUCCEEDED"); }));
}

TEST_F(TacticalTest, MissingCoverOrDefenseGoalFails)
{
  pb_rm_interfaces::msg::GroundRobotPosition missing; set("referee_groundRobotPosition", missing);
  choose(1); ASSERT_TRUE(until([&] { return has(1, "FAILED"); }));
  pb_rm_interfaces::msg::EventData event; event.base_hp = 10000; set("referee_eventData", event);
  choose(4); ASSERT_TRUE(until([&] { return has(4, "FAILED"); })); EXPECT_TRUE(requested.empty());
}

TEST_F(TacticalTest, GatherMissingLockCompletesAfterWait)
{
  choose(6); arrive(); run(30); EXPECT_FALSE(has(6, "SUCCEEDED"));
  ASSERT_TRUE(until([&] { return has(6, "SUCCEEDED"); }));
}

TEST_F(TacticalTest, GatherContinuousLockLossAndStaleness)
{
  choose(6); arrive(); std_msgs::msg::Bool locked; locked.data = true;
  for (int i = 0; i < 70; ++i) { set("auto_aim_target_locked", locked); step(); }
  EXPECT_FALSE(has(6, "SUCCEEDED"));
  run(100); EXPECT_FALSE(has(6, "SUCCEEDED"));
  ASSERT_TRUE(until([&] { return has(6, "SUCCEEDED"); }));
}

TEST_F(TacticalTest, LockReacquisitionResetsNoEnemyTimer)
{
  choose(6); arrive(); std_msgs::msg::Bool locked; locked.data = true;
  set("auto_aim_target_locked", locked); step(); locked.data = false;
  set("auto_aim_target_locked", locked); run(70); EXPECT_FALSE(has(6, "SUCCEEDED"));
  locked.data = true; set("auto_aim_target_locked", locked); step();
  locked.data = false; set("auto_aim_target_locked", locked); run(70);
  EXPECT_FALSE(has(6, "SUCCEEDED"));
  ASSERT_TRUE(until([&] { return has(6, "SUCCEEDED"); }));
}

TEST_F(TacticalTest, PriorityAndRestartFromFirstCapturePoint)
{
  choose(5); arrive(); ASSERT_TRUE(until([&] { return handles.size() == 2; }));
  choose(4); ASSERT_TRUE(until([&] { return handles.size() == 3; }));
  choose(3); ASSERT_TRUE(until([&] { return handles.size() == 4; }));
  choose(4); ASSERT_TRUE(until([&] { return handles.size() == 5; }));
  choose(0); ASSERT_TRUE(until([&] { return has(0, "STARTED"); }));
  choose(5); ASSERT_TRUE(until([&] { return handles.size() == 6; }));
  EXPECT_EQ(requested.back().pose.pose.position.x, 1.0);
  EXPECT_TRUE(has(5, "PREEMPTED")); EXPECT_TRUE(has(4, "PREEMPTED"));
  EXPECT_TRUE(has(3, "PREEMPTED"));
}

TEST_F(TacticalTest, StaleDecisionStopsNavigation)
{
  choose(6); ASSERT_TRUE(until([&] { return handles.size() == 1; }));
  bb->set("tactical_decision_stamp_ns", int64_t{1}); run(60, false);
  EXPECT_TRUE(has(6, "PREEMPTED")); EXPECT_TRUE(has(0, "STARTED"));
}

TEST_F(TacticalTest, RejectedAndAbortedNavigation)
{
  reject = true; choose(6); ASSERT_TRUE(until([&] { return has(6, "FAILED"); }));
  reject = false; choose(5); ASSERT_TRUE(until([&] { return handles.size() == 1; }));
  handles[0]->abort(std::make_shared<Nav::Action::Result>());
  ASSERT_TRUE(until([&] { return has(5, "FAILED"); }));
}

TEST_F(TacticalTest, NavigationTimeoutCancelsBeforeFailure)
{
  set("navigation_timeout_ms", 60); choose(6);
  ASSERT_TRUE(until([&] { return has(6, "FAILED"); })); EXPECT_EQ(cancels, 1);
}

TEST_F(TacticalTest, UnconfirmedCancellationBlocksReplacement)
{
  choose(6); ASSERT_TRUE(until([&] { return handles.size() == 1; }));
  complete_cancel = false; choose(3); run(150);
  EXPECT_EQ(requested.size(), 1U); EXPECT_FALSE(has(3, "STARTED"));
  bool reported = false;
  for (const auto & e : events) { reported |= e.reason == "navigation_cancel_unconfirmed"; }
  EXPECT_TRUE(reported);
  complete_cancel = true;
  ASSERT_TRUE(until([&] { return handles.size() == 2; }));
}

TEST_F(TacticalTest, DelayedAcceptanceIsCanceledBeforeNewGoal)
{
  choose(6); tree.tickOnce(); choose(3); tree.tickOnce();
  ASSERT_TRUE(until([&] { return handles.size() == 2; }));
  EXPECT_EQ(cancels, 1); EXPECT_TRUE(has(6, "PREEMPTED"));
  EXPECT_FALSE(has(3, "FAILED"));
}

TEST_F(TacticalTest, RejectedCancellationWaitsForActualTerminalResult)
{
  choose(6); ASSERT_TRUE(until([&] { return handles.size() == 1; }));
  reject_cancel = true; choose(3); run(120);
  EXPECT_EQ(requested.size(), 1U); EXPECT_FALSE(has(3, "STARTED"));
  handles[0]->succeed(std::make_shared<Nav::Action::Result>());
  ASSERT_TRUE(until([&] { return handles.size() == 2; }));
  EXPECT_TRUE(has(6, "PREEMPTED")); EXPECT_FALSE(has(6, "SUCCEEDED"));
}

TEST_F(TacticalTest, RestartedTreePreservesCancellationBarrier)
{
  choose(6); ASSERT_TRUE(until([&] { return handles.size() == 1; }));
  complete_cancel = false;
  tree.haltTree();
  tree = factory.createTree("sentry_tactical", bb);
  choose(3); run(100); EXPECT_EQ(requested.size(), 1U);
  complete_cancel = true;
  ASSERT_TRUE(until([&] { return handles.size() == 2; }));
}

TEST_F(TacticalTest, SameActionWithNewDecisionDoesNotRestartDwell)
{
  choose(4); arrive();
  for (int i = 0; i < 15; ++i) { choose(4); step(); }
  ASSERT_TRUE(until([&] { return has(4, "SUCCEEDED"); }));
  EXPECT_EQ(requested.size(), 1U);
}

TEST_F(TacticalTest, WriteModels)
{
  if (const char * destination = std::getenv("TACTICAL_MODEL_OUTPUT")) {
    std::ofstream file(destination); file << BT::writeTreeNodesModelXML(factory);
  }
  EXPECT_EQ(factory.registeredBehaviorTrees().size(), 8U);
}

TEST_F(TacticalTest, OrdinarySwitchUsesLatestDecisionAfterCancel)
{
  choose(6); ASSERT_TRUE(until([&] { return handles.size() == 1; }));
  complete_cancel = false;
  choose(5); run(40);
  choose(2); run(40);
  EXPECT_EQ(requested.size(), 1U);
  complete_cancel = true;
  ASSERT_TRUE(until([&] { return has(2, "STARTED"); }));
  EXPECT_FALSE(has(5, "STARTED"));
  EXPECT_TRUE(has(6, "PREEMPTED"));
}

TEST_F(TacticalTest, OrdinaryDefenseAcceptsWarningBand)
{
  pb_rm_interfaces::msg::EventData event;
  event.base_hp = 5100;
  event.fortress_gain_zone = event.OCCUPIED_FRIEND;
  set("referee_eventData", event);
  choose(4);
  ASSERT_TRUE(until([&] { return handles.size() == 1; }));
  EXPECT_FALSE(has(4, "FAILED"));
}

TEST_F(TacticalTest, FeedbackFloodDoesNotStarveGoalResponse)
{
  // Queue many feedback samples before polling, as real Nav2 does at > BT rate.
  choose(6); tree.tickOnce(); rclcpp::spin_some(node);
  ASSERT_EQ(handles.size(), 1U);
  for (int i = 0; i < 30; ++i) {
    handles[0]->publish_feedback(std::make_shared<Nav::Action::Feedback>());
  }
  choose(3);
  ASSERT_TRUE(until([&] { return has(3, "STARTED"); }));
  EXPECT_EQ(cancels, 1);
  EXPECT_TRUE(has(6, "PREEMPTED"));
}

// Copyright 2026 RMUC contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "pb2025_sentry_behavior/tactical_scorer_node.hpp"

using namespace std::chrono_literals;
using Decision = pb2025_sentry_decision_interfaces::msg::TacticalDecision;
using Execution = pb2025_sentry_decision_interfaces::msg::TacticalExecution;

class ScorerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    io = std::make_shared<rclcpp::Node>("scorer_test_io");
    mapping = io->create_publisher<std_msgs::msg::Bool>(
      "/mapping_bootstrap_finished", rclcpp::QoS(1).transient_local().reliable());
    game_pub = io->create_publisher<pb_rm_interfaces::msg::GameStatus>("referee/game_status", 10);
    robot_pub = io->create_publisher<pb_rm_interfaces::msg::RobotStatus>("referee/robot_status", 10);
    hp_pub = io->create_publisher<pb_rm_interfaces::msg::GameRobotHP>("referee/all_robot_hp", 10);
    event_pub = io->create_publisher<pb_rm_interfaces::msg::EventData>("referee/event_data", 10);
    positions_pub = io->create_publisher<pb_rm_interfaces::msg::GroundRobotPosition>(
      "referee/ground_robot_position", 10);
    target_pub = io->create_publisher<auto_aim_interfaces::msg::Target>("tracker/target", 10);
    feedback = io->create_publisher<Execution>("/tactical_execution", 10);
    simulation_pub = io->create_publisher<std_msgs::msg::String>("/referee/simulation_state", 10);
    decisions = io->create_subscription<Decision>(
      "/tactical_decision", 10, [this](Decision::SharedPtr msg) {received.push_back(*msg);});

    game.game_progress = game.RUNNING;
    game.stage_remain_time = 400;
    robot.current_hp = 400;
    robot.maximum_hp = 400;
    robot.projectile_allowance_17mm = 150;
    robot.shooter_barrel_heat_limit = 400;
    robot.shooter_17mm_1_barrel_heat = 40;
    event.base_hp = 10000;
    event.fortress_gain_zone = event.OCCUPIED_FRIEND;
    event.center_gain_zone = event.OCCUPIED_FRIEND;
    positions.standard_3_position.x = 2.0;
    positions.standard_3_position.y = 1.0;
    executor.add_node(io);
  }

  void start(std::vector<rclcpp::Parameter> overrides = {})
  {
    std::vector<rclcpp::Parameter> parameters = {
      rclcpp::Parameter("tick_hz", 50.0),
      rclcpp::Parameter("action_failure_cooldown_ms", 500),
      rclcpp::Parameter("decision_hold_ms", 0),
      rclcpp::Parameter("action_switch_margin", 0.0)};
    parameters.insert(parameters.end(), overrides.begin(), overrides.end());
    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);
    scorer = std::make_shared<pb2025_sentry_behavior::TacticalScorerNode>(options);
    executor.add_node(scorer);
    run(100);
    ASSERT_FALSE(received.empty());
  }

  void run(int milliseconds)
  {
    const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(milliseconds);
    while (std::chrono::steady_clock::now() < until) {
      if (publish_game) {game_pub->publish(game);}
      if (publish_robot) {robot_pub->publish(robot);}
      if (publish_hp) {hp_pub->publish(hp);}
      if (publish_event) {event_pub->publish(event);}
      if (publish_positions) {positions_pub->publish(positions);}
      if (publish_target) {target_pub->publish(target);}
      executor.spin_some();
      std::this_thread::sleep_for(2ms);
    }
    executor.spin_some();
  }

  const Decision & lastDecision() const {return received.back();}

  bool hasText(const std::vector<std::string> & values, const std::string & text) const
  {
    return std::any_of(values.begin(), values.end(), [&](const std::string & value) {
      return value.find(text) != std::string::npos;
    });
  }

  std::string diagnostic(const std::string & action) const
  {
    const std::string prefix = action + ":";
    for (const auto * entries : {&lastDecision().top_reasons, &lastDecision().rejected_actions}) {
      for (const auto & entry : *entries) {
        if (entry.rfind(prefix, 0) == 0) {return entry;}
      }
    }
    ADD_FAILURE() << "Missing diagnostic for " << action;
    return "";
  }

  double factor(const std::string & action, const std::string & name) const
  {
    const auto entry = diagnostic(action);
    const auto pos = entry.find(name + "=");
    if (pos == std::string::npos) {
      ADD_FAILURE() << "Missing factor " << name << " in " << entry;
      return std::numeric_limits<double>::quiet_NaN();
    }
    return std::stod(entry.substr(pos + name.size() + 1));
  }

  Execution failCurrent()
  {
    Execution failure;
    failure.action = lastDecision().action;
    failure.decision_id = lastDecision().decision_id;
    failure.state = "FAILED";
    feedback->publish(failure);
    run(60);
    return failure;
  }

  rclcpp::Node::SharedPtr io;
  std::shared_ptr<pb2025_sentry_behavior::TacticalScorerNode> scorer;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mapping;
  rclcpp::Publisher<pb_rm_interfaces::msg::GameStatus>::SharedPtr game_pub;
  rclcpp::Publisher<pb_rm_interfaces::msg::RobotStatus>::SharedPtr robot_pub;
  rclcpp::Publisher<pb_rm_interfaces::msg::GameRobotHP>::SharedPtr hp_pub;
  rclcpp::Publisher<pb_rm_interfaces::msg::EventData>::SharedPtr event_pub;
  rclcpp::Publisher<pb_rm_interfaces::msg::GroundRobotPosition>::SharedPtr positions_pub;
  rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub;
  rclcpp::Publisher<Execution>::SharedPtr feedback;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr simulation_pub;
  rclcpp::Subscription<Decision>::SharedPtr decisions;
  std::vector<Decision> received;
  pb_rm_interfaces::msg::GameStatus game;
  pb_rm_interfaces::msg::RobotStatus robot;
  pb_rm_interfaces::msg::GameRobotHP hp;
  pb_rm_interfaces::msg::EventData event;
  pb_rm_interfaces::msg::GroundRobotPosition positions;
  auto_aim_interfaces::msg::Target target;
  bool publish_game{true};
  bool publish_robot{true};
  bool publish_hp{true};
  bool publish_event{true};
  bool publish_positions{true};
  bool publish_target{true};
};

TEST_F(ScorerTest, DefaultsToGatherInfoAndUsesNewDiagnostics)
{
  start();
  EXPECT_EQ(lastDecision().action, Decision::GATHER_INFO);
  EXPECT_NEAR(factor("GATHER_INFO", "importance"), 0.58, 1e-6);
  EXPECT_NEAR(factor("GATHER_INFO", "information_gap"), 0.90, 1e-6);
  EXPECT_NEAR(factor("GATHER_INFO", "target_absence"), 1.0, 1e-6);
  EXPECT_NEAR(lastDecision().score, 0.58 * std::sqrt(0.9), 1e-6);
}

TEST_F(ScorerTest, CriticalResourcesAndBaseDangerRemainHardRules)
{
  start();
  robot.current_hp = 100;
  run(60);
  EXPECT_EQ(lastDecision().action, Decision::RESUPPLY_HOME);
  EXPECT_TRUE(lastDecision().emergency);

  robot.current_hp = 400;
  event.base_hp = 5000;
  run(60);
  EXPECT_EQ(lastDecision().action, Decision::DEFEND_HOME);
  EXPECT_TRUE(lastDecision().emergency);
}

TEST_F(ScorerTest, AttackUsesOutpostOpportunityAndResourceReadiness)
{
  hp.red_outpost_hp = 1500;
  start();
  EXPECT_EQ(lastDecision().action, Decision::ATTACK_OUTPOST);
  EXPECT_NEAR(factor("ATTACK_OUTPOST", "outpost_opportunity"), 0.5 + 0.5 * (1.0 - 1500.0 / 1800.0), 1e-6);
  EXPECT_NEAR(factor("ATTACK_OUTPOST", "resource_ready"), 1.0, 1e-6);
  const auto expected = 0.78 * std::sqrt(factor("ATTACK_OUTPOST", "outpost_opportunity"));
  EXPECT_NEAR(lastDecision().score, expected, 1e-6);

  target.tracking = false;
  publish_target = false;
  run(100);
  EXPECT_EQ(lastDecision().action, Decision::ATTACK_OUTPOST);
}

TEST_F(ScorerTest, ResourceNeedUsesGeometricMeanWithoutCriticalPreemption)
{
  start();
  robot.current_hp = 120;
  run(60);
  EXPECT_EQ(lastDecision().action, Decision::RESUPPLY_HOME);
  EXPECT_FALSE(lastDecision().emergency);
  EXPECT_NEAR(factor("RESUPPLY_HOME", "resource_need"), 0.75, 1e-6);
  EXPECT_NEAR(factor("RESUPPLY_HOME", "criticality"), 0.75, 1e-6);
  EXPECT_NEAR(lastDecision().score, 0.90 * 0.75, 1e-6);
}

TEST_F(ScorerTest, DefenseThreatRequiresThreeFreshObservationsForPersistence)
{
  event.base_hp = 6000;
  start();
  EXPECT_EQ(lastDecision().action, Decision::GATHER_INFO);
  EXPECT_EQ(factor("DEFEND_HOME", "score"), 0.0);
  publish_event = false;
  event.fortress_gain_zone = event.OCCUPIED_ENEMY;
  for (int count = 1; count <= 3; ++count) {
    event_pub->publish(event);
    run(50);
    EXPECT_NEAR(factor("DEFEND_HOME", "threat_persistence"), count / 3.0, 1e-5);
    EXPECT_FALSE(lastDecision().emergency);
  }
  EXPECT_EQ(lastDecision().action, Decision::DEFEND_HOME);
  run(1050);
  EXPECT_EQ(lastDecision().action, Decision::HOLD_SAFE);
  EXPECT_EQ(factor("DEFEND_HOME", "threat_persistence"), 0.0);
  event_pub->publish(event);
  run(50);
  EXPECT_NEAR(factor("DEFEND_HOME", "threat_persistence"), 1.0 / 3.0, 1e-5);
  event.fortress_gain_zone = event.OCCUPIED_FRIEND;
  event_pub->publish(event);
  run(50);
  EXPECT_EQ(factor("DEFEND_HOME", "threat_persistence"), 0.0);
}

TEST_F(ScorerTest, CaptureUsesPointValueAndMatchPhase)
{
  hp.red_outpost_hp = 0;
  event.center_gain_zone = event.OCCUPIED_ENEMY;
  game.stage_remain_time = 20;
  start();
  EXPECT_EQ(lastDecision().action, Decision::CAPTURE_POINT);
  EXPECT_NEAR(factor("CAPTURE_POINT", "point_value"), 1.0, 1e-6);
  EXPECT_NEAR(factor("CAPTURE_POINT", "match_phase"), 1.0, 1e-6);
  game.stage_remain_time = 200;
  run(60);
  EXPECT_NEAR(factor("CAPTURE_POINT", "match_phase"), 0.70, 1e-6);
}

TEST_F(ScorerTest, CoverUsesTeammateInjuryOnly)
{
  hp.red_outpost_hp = 0;
  hp.blue_3_robot_hp = 10;
  start();
  EXPECT_EQ(lastDecision().action, Decision::COVER_TEAMMATE);
  EXPECT_NEAR(factor("COVER_TEAMMATE", "teammate_injury"), 0.96, 1e-6);
}

TEST_F(ScorerTest, FailureCooldownRemovesActionFromCandidates)
{
  hp.red_outpost_hp = 1500;
  start();
  ASSERT_EQ(lastDecision().action, Decision::ATTACK_OUTPOST);
  const auto failure = failCurrent();
  EXPECT_EQ(lastDecision().action, Decision::GATHER_INFO);
  EXPECT_EQ(factor("ATTACK_OUTPOST", "score"), 0.0);
  EXPECT_TRUE(hasText(lastDecision().rejected_actions, "ATTACK_OUTPOST:failure_cooldown"));
  feedback->publish(failure);
  run(60);
  EXPECT_EQ(lastDecision().action, Decision::GATHER_INFO);
}

TEST_F(ScorerTest, MissingRequiredDataHoldsSafely)
{
  publish_game = false;
  start();
  EXPECT_EQ(lastDecision().action, Decision::HOLD_SAFE);
  EXPECT_TRUE(lastDecision().emergency);
  EXPECT_TRUE(hasText(lastDecision().top_reasons, "missing_game_status"));
}

TEST_F(ScorerTest, RejectsInvalidImportanceConfiguration)
{
  for (const auto & parameter : {
      rclcpp::Parameter("action_importance", std::vector<double>{0.1}),
      rclcpp::Parameter("action_importance", std::vector<double>(7, 1.1)),
      rclcpp::Parameter("outpost_reference_hp", 0)}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({parameter});
    EXPECT_THROW(std::make_shared<pb2025_sentry_behavior::TacticalScorerNode>(options),
      std::invalid_argument);
  }
}

TEST_F(ScorerTest, NonRunningPublishesHoldAndRecovers)
{
  start();
  for (auto phase : {game.NOT_START, game.GAME_OVER}) {
    const auto count = received.size();
    game.game_progress = phase;
    run(60);
    EXPECT_GT(received.size(), count);
    EXPECT_EQ(lastDecision().action, Decision::HOLD_SAFE);
    EXPECT_TRUE(hasText(lastDecision().top_reasons, "game_not_running"));
  }
  game.game_progress = game.RUNNING;
  run(60);
  EXPECT_EQ(lastDecision().action, Decision::GATHER_INFO);
}

TEST_F(ScorerTest, WarningBandAndCriticalBaseAreExecutableCandidates)
{
  event.base_hp = 5100;
  start();
  EXPECT_EQ(lastDecision().action, Decision::DEFEND_HOME);
  EXPECT_FALSE(lastDecision().emergency);
  EXPECT_NEAR(factor("DEFEND_HOME", "threat_level"), 0.8, 1e-5);
  event.base_hp = 5000;
  run(60);
  EXPECT_TRUE(lastDecision().emergency);
  EXPECT_NEAR(factor("DEFEND_HOME", "score"), .82, 1e-5);
  event.base_hp = 5500;
  run(60);
  EXPECT_EQ(lastDecision().action, Decision::GATHER_INFO);
}

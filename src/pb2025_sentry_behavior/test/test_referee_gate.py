#!/usr/bin/env python3
# Copyright 2026 RMUC contributors
# Licensed under the Apache License, Version 2.0.
"""Referee state and command gate regressions; no physical command topics used."""
import importlib.util
import math
from pathlib import Path
import sys
import time
import unittest

import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from pb2025_sentry_decision_interfaces.msg import TacticalDecision

scripts = Path(__file__).resolve().parents[1] / 'scripts'


def load(name):
    spec = importlib.util.spec_from_file_location(name, scripts / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


Simulator = load('sentry_referee_simulator').SentryRefereeSimulator
Gate = load('sentry_test_velocity_gate').TestVelocityGate


class RefereeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=['--ros-args', '-p', 'publish_target:=false'])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Simulator()

    def tearDown(self):
        self.node.destroy_node()

    def set_parameters(self, **values):
        return self.node.set_parameters_atomically([Parameter(k, value=v) for k, v in values.items()])

    def test_default_hold_and_real_vision_ownership(self):
        self.assertEqual(self.node.scenario, 'hold_safe')
        self.assertIsNone(self.node.target_pub)
        self.node.on_timer()
        self.assertEqual(self.node.world_state_id, 1)

    def test_invalid_group_does_not_partially_apply(self):
        result = self.set_parameters(scenario='defend_base', hp=99999)
        self.assertFalse(result.successful)
        self.assertEqual(self.node.get_parameter('scenario').value, 'hold_safe')
        self.assertFalse(self.set_parameters(scenario='typo').successful)
        self.assertFalse(self.set_parameters(paused_topics='unknown').successful)

    def test_overrides_consistent_and_pause(self):
        self.assertTrue(self.set_parameters(hp=100, base_hp=5100, outpost_hp=800,
                                           teammate_hp=20, paused_topics='event_data').successful)
        state = self.node.base_state('test')
        self.node.apply_overrides(state)
        self.assertEqual(state.robot_status.robot_id, 107)
        self.assertEqual(state.all_robot_hp.blue_7_robot_hp, 100)
        self.assertEqual(state.all_robot_hp.blue_base_hp, 5100)
        self.assertEqual(state.all_robot_hp.red_outpost_hp, 800)
        self.assertEqual(state.all_robot_hp.blue_3_robot_hp, 20)
        self.node.on_timer()

    def test_game_over_keeps_timer_and_can_restart(self):
        self.assertTrue(self.set_parameters(scenario='game_over').successful)
        self.node.on_timer()
        self.assertFalse(self.node.timer.is_canceled())
        self.assertTrue(self.set_parameters(scenario='match').successful)
        self.node.on_timer()
        self.assertEqual(self.node.build_match_state().game_status.game_progress, 1)
        self.assertTrue(self.set_parameters(scenario='gather_info').successful)
        self.node.on_timer()
        self.assertEqual(self.node.scenario, 'gather_info')


class GateTest(unittest.TestCase):
    def setUp(self):
        # Exercise the pure policy without constructing a ROS node/publisher.
        self.gate = Gate.__new__(Gate)
        self.gate.stamps = dict(command=10., upstream=10., odom=10., decision=10.)
        self.gate.decision = TacticalDecision(action=6)
        self.gate.navigating = True
        self.gate.stop_until = 0.
        self.gate.command = Twist()
        self.gate.command.linear.x = self.gate.command.linear.y = 1.
        self.gate.command.angular.z = 2.

    def test_vector_and_angular_limits(self):
        cmd, reason = self.gate.safe_command(10.1, True)
        self.assertEqual(reason, 'enabled')
        self.assertAlmostEqual(math.hypot(cmd.linear.x, cmd.linear.y), .3)
        self.assertEqual(cmd.angular.z, .5)

    def test_default_disabled_and_all_watchdogs(self):
        self.assertEqual(self.gate.safe_command(10.1, False)[1], 'disabled')
        for key in self.gate.stamps:
            saved = self.gate.stamps[key]
            self.gate.stamps[key] = 8.
            self.assertEqual(self.gate.safe_command(10.1, True)[1], 'stale_' + key)
            self.gate.stamps[key] = saved

    def test_stop_and_cancel_and_nonfinite(self):
        self.gate.stop_until = 10.3
        self.assertEqual(self.gate.safe_command(10.1, True)[1], 'stop_requested')
        self.gate.stop_until = 0.
        self.gate.navigating = False
        self.assertEqual(self.gate.safe_command(10.1, True)[1], 'navigation_not_active')
        self.gate.navigating = True
        self.gate.command.linear.x = float('nan')
        self.assertEqual(self.gate.safe_command(10.1, True)[1], 'invalid_velocity')


if __name__ == '__main__':
    unittest.main()

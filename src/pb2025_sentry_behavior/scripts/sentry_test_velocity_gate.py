#!/usr/bin/env python3
# Copyright 2026 RMUC contributors
# Licensed under the Apache License, Version 2.0.
"""Opt-in hardware test gate, after velocity frame conversion and before serial."""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pb2025_sentry_decision_interfaces.msg import TacticalDecision
from rcl_interfaces.msg import SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class TestVelocityGate(Node):
    def __init__(self):
        super().__init__('sentry_test_velocity_gate')
        self.declare_parameter('enabled', False)
        self.add_on_set_parameters_callback(self.validate)
        self.command = Twist()
        self.stamps = dict(command=0., upstream=0., odom=0., decision=0.)
        self.decision = None
        self.navigating = False
        self.stop_until = 0.
        self.last_reason = None
        self.output_topic = self.declare_parameter('output_topic', '/cmd_vel_base_real_yaw').value
        self.output = self.create_publisher(Twist, self.output_topic, 10)
        self.status = self.create_publisher(String, '/test_velocity_gate/status', 10)
        self.create_subscription(Twist, '/test_velocity/converted', self.on_command, 10)
        # The transform node can replay cached commands on odometry updates.
        # Require a fresh ORIGINAL command as well as a transformed command.
        self.create_subscription(Twist, '/cmd_vel_nav2_result', self.on_upstream, 10)
        self.create_subscription(Odometry, '/odometry', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(TacticalDecision, '/tactical_decision', self.on_decision, 10)
        self.create_subscription(Twist, '/safety_stop', self.on_stop, 10)
        self.create_subscription(String, '/tactical_navigation_status', self.on_navigation, 10)
        self.timer = self.create_timer(.05, self.control,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))

    def validate(self, parameters):
        for parameter in parameters:
            if parameter.name != 'enabled' or type(parameter.value) is not bool:
                return SetParametersResult(successful=False, reason='Only enabled (bool) is mutable')
        return SetParametersResult(successful=True)

    def on_command(self, message):
        self.command = message
        self.stamps['command'] = time.monotonic()

    def on_upstream(self, _message):
        self.stamps['upstream'] = time.monotonic()

    def on_odom(self, message):
        p = message.pose.pose.position
        if all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self.stamps['odom'] = time.monotonic()

    def on_decision(self, message):
        self.decision = message
        self.stamps['decision'] = time.monotonic()

    def on_stop(self, _message):
        self.stop_until = time.monotonic() + .3
        self.output.publish(Twist())

    def on_navigation(self, message):
        state = message.data.split(' state=', 1)[-1]
        if state == 'accepted':
            self.navigating = True
        elif state in ('goal_sent', 'cancel_requested', 'cancel_sent') or state.startswith('navigation_'):
            self.navigating = False
            self.output.publish(Twist())

    def safe_command(self, now, enabled):
        if not enabled:
            return Twist(), 'disabled'
        if now < self.stop_until:
            return Twist(), 'stop_requested'
        for key, timeout in (('decision', 1.), ('odom', .5), ('upstream', .5), ('command', .5)):
            if now - self.stamps[key] > timeout or self.stamps[key] <= 0:
                return Twist(), f'stale_{key}'
        if self.decision is None or not 1 <= self.decision.action <= 6:
            return Twist(), 'hold_safe'
        if not self.navigating:
            return Twist(), 'navigation_not_active'
        values = (self.command.linear.x, self.command.linear.y, self.command.angular.z)
        if not all(math.isfinite(value) for value in values):
            return Twist(), 'invalid_velocity'
        output = Twist()
        scale = min(1., .3 / max(math.hypot(*values[:2]), 1e-12))
        output.linear.x, output.linear.y = values[0] * scale, values[1] * scale
        output.angular.z = max(-.5, min(.5, values[2]))
        return output, 'enabled'

    def control(self):
        command, reason = self.safe_command(time.monotonic(), self.get_parameter('enabled').value)
        # A second final-velocity publisher bypasses this gate: fail closed and
        # expose the topology fault instead of silently racing commands.
        if self.count_publishers(self.output_topic) != 1 or self.count_publishers('/test_disabled_spin'):
            command, reason = Twist(), 'publisher_conflict'
        self.output.publish(command)
        if reason != self.last_reason:
            self.get_logger().info(f'[VelocityGate] {reason}')
            self.last_reason = reason
        self.status.publish(String(data=reason))


def main():
    rclpy.init()
    node = TestVelocityGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.output.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Run only against an isolated Gazebo tactical stack; never a physical robot."""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rcl_interfaces.srv import SetParametersAtomically
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from pb2025_sentry_decision_interfaces.msg import TacticalDecision, TacticalExecution


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cycles', type=int, default=20)
    parser.add_argument('--output', default='/tmp/sentry_transition_results.json')
    args = parser.parse_args()
    rclpy.init()
    node = Node('tactical_transition_probe')
    decisions, executions, nav, poses = [], [], [], []
    node.create_subscription(TacticalDecision, '/tactical_decision', decisions.append, 10)
    node.create_subscription(TacticalExecution, '/tactical_execution', executions.append, 100)
    node.create_subscription(String, '/tactical_navigation_status',
                             lambda m: nav.append((time.monotonic(), m.data)), 100)
    node.create_subscription(Odometry, '/odom', poses.append, 10)
    client = node.create_client(SetParametersAtomically,
                               '/sentry_referee_simulator/set_parameters_atomically')
    report = {'cycles': [], 'cancel_seconds': [], 'passed': False}

    def wait(predicate, timeout=15):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
            if predicate():
                return
        raise AssertionError('Timeout; last decision/execution: ' +
                             str((decisions[-1] if decisions else None,
                                  executions[-1] if executions else None)))

    def scenario(name, action):
        future = client.call_async(SetParametersAtomically.Request(parameters=[
            Parameter(name='scenario', value=ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value=name))]))
        wait(future.done)
        assert future.result().result.successful, future.result()
        count = len(executions)
        wait(lambda: decisions and decisions[-1].action == action)
        if name != 'hold_safe':
            wait(lambda: any(m.action == action and m.state == 'STARTED'
                             for m in executions[count:]), 12)
        else:
            wait(lambda: poses and math.hypot(poses[-1].twist.twist.linear.x,
                                             poses[-1].twist.twist.linear.y) < .03, 3)
        print(name, 'OK', flush=True)

    try:
        wait(lambda: client.service_is_ready() and bool(poses), 30)
        scenario('hold_safe', 0)
        start = poses[-1].pose.pose.position
        scenario('gather_info', 6)
        scenario('defend_base', 4)
        wait(lambda: math.hypot(poses[-1].pose.pose.position.x - start.x,
                               poses[-1].pose.pose.position.y - start.y) > .3, 20)
        for cycle in range(args.cycles):
            for name, action in [('gather_info', 6), ('defend_base', 4),
                                 ('resupply_home', 3), ('capture_point', 5), ('hold_safe', 0)]:
                scenario(name, action)
            report['cycles'].append(cycle + 1)
        pending = {}
        for stamp, entry in nav:
            generation = entry.split()[0]
            if entry.endswith('state=cancel_requested'):
                pending[generation] = stamp
            elif 'state=navigation_' in entry and generation in pending:
                report['cancel_seconds'].append(stamp - pending.pop(generation))
        assert not pending, pending
        assert report['cancel_seconds'], 'No cancellations exercised'
        assert max(report['cancel_seconds']) <= 2., report['cancel_seconds']
        assert not any('unconfirmed' in e.reason for e in executions)
        report['passed'] = True
    finally:
        Path(args.output).write_text(json.dumps(report, indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

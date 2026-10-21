# Copyright 2026 RMUC contributors
# Licensed under the Apache License, Version 2.0.
"""Smoke-test the decision-only entry point with no navigation or referee publisher."""
import argparse
import signal
import subprocess
import tempfile
import time

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSProfile, DurabilityPolicy
from action_msgs.msg import GoalStatusArray, GoalStatus
from action_msgs.srv import CancelGoal


def main():
    """Assert the launch starts exactly the decision processes and safe defaults."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--launch', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('decision_launch_probe')
    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen(['ros2', 'launch', args.launch], stdout=log, stderr=log)
        try:
            client = node.create_client(GetParameters, '/pb2025_sentry_behavior_server/get_parameters')
            deadline = time.monotonic() + 12
            while not client.service_is_ready() and time.monotonic() < deadline:
                assert process.poll() is None, 'launch terminated early'
                rclpy.spin_once(node, timeout_sec=0.1)
            assert client.service_is_ready(), 'decision server did not start'
            future = client.call_async(GetParameters.Request(names=['use_sim_time', 'tick_frequency',
                                            'navigation_timeout_ms',
                                            'navigation_cancel_timeout_ms',
                                            'navigation_server_wait_timeout_ms',
                                            'referee_data_timeout_ms',
                                            'navigation_action_name',
                                            'validate_navigation_goals']))
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
            assert future.done(), 'parameter request timed out'
            values = future.result().values
            assert not values[0].bool_value
            assert values[1].integer_value == 20
            assert values[2].integer_value == 180000
            assert values[3].integer_value == 2000
            assert values[4].integer_value == 10000
            assert values[5].integer_value == 3000
            assert values[6].string_value == '/navigate_to_pose'
            assert values[7].bool_value
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            names = set(node.get_node_names())
            expected = {'tactical_scorer_node', 'pb2025_sentry_behavior_server',
                        'pb2025_sentry_behavior_client'}
            assert expected <= names, names
            assert not {'controller_server', 'planner_server', 'bt_navigator'} & names, names
            assert node.count_publishers('/cmd_vel') == 0, 'decision publishes velocity'
            statuses = []
            node.create_subscription(
                GoalStatusArray, '/pb2025_sentry_behavior/_action/status', statuses.append,
                QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            cancel = node.create_client(CancelGoal, '/pb2025_sentry_behavior/_action/cancel_goal')
            assert cancel.wait_for_service(timeout_sec=2)
            future = cancel.call_async(CancelGoal.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
            assert future.done() and future.result().goals_canceling, 'tree was not active'
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            assert statuses and statuses[-1].status_list
            assert all(item.status == GoalStatus.STATUS_CANCELED
                       for item in statuses[-1].status_list), 'client restarted canceled tree'
            print('PASS: decision-only launch, real-time defaults, map validation, no velocity')
        except BaseException:
            log.seek(0)
            print(log.read())
            raise
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Live standard-action acceptance test; run against an isolated fresh backend."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState


def main():
    rclpy.init()
    node = Node('nav2_backend_probe')
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    positions = []
    node.create_subscription(Odometry, '/odom', positions.append, 10)

    def wait(predicate, seconds=60):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError('Timed out waiting for navigation condition')

    def position():
        p = positions[-1].pose.pose.position
        return p.x, p.y

    def send(x, y):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.
        future = client.send_goal_async(goal)
        wait(future.done)
        handle = future.result()
        assert handle.accepted, 'Nav2 rejected goal'
        print('ACCEPTED', x, y, flush=True)
        return handle

    def arrive(x, y):
        result = send(x, y).get_result_async()
        wait(result.done)
        assert result.result().status == GoalStatus.STATUS_SUCCEEDED
        assert math.dist(position(), (x, y)) < .3
        print('SUCCEEDED', position(), flush=True)

    try:
        wait(lambda: client.server_is_ready() and bool(positions))
        lifecycle = node.create_client(GetState, '/bt_navigator/get_state')
        wait(lifecycle.service_is_ready)
        active = False
        deadline = time.monotonic() + 40
        while not active and time.monotonic() < deadline:
            state = lifecycle.call_async(GetState.Request())
            wait(state.done)
            active = state.result().current_state.id == 3
            rclpy.spin_once(node, timeout_sec=.2)
        assert active, 'Nav2 not active'
        names = {name for name, _ in node.get_node_names_and_namespaces()}
        assert not any('scorer' in n or 'referee' in n or 'sentry_behavior' in n for n in names)
        print('NO TACTICAL NODES', sorted(names), flush=True)
        start = position()
        handle = send(5, 7)
        result = handle.get_result_async()
        wait(lambda: math.dist(position(), start) > .6)
        print('MOVING', position(), flush=True)
        cancel = handle.cancel_goal_async()
        wait(cancel.done)
        assert cancel.result().goals_canceling
        wait(result.done)
        assert result.result().status == GoalStatus.STATUS_CANCELED
        wait(lambda: math.hypot(positions[-1].twist.twist.linear.x, positions[-1].twist.twist.linear.y) < .03)
        print('CANCELED AND STOPPED', position(), flush=True)
        arrive(5, 7)
        arrive(5, 9.5)
        result = send(100, 100).get_result_async()
        wait(result.done, 90)
        assert result.result().status == GoalStatus.STATUS_ABORTED
        print('UNREACHABLE ABORTED', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

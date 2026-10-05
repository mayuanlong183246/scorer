#!/usr/bin/env python3
"""Interface fixture only: no physics, no Gazebo; never use for navigation acceptance."""
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry


class Fixture(Node):
    def __init__(self):
        super().__init__('mock_navigation_interface_only')
        self.declare_parameter('outcome', 'succeed')
        self.odom = self.create_publisher(Odometry, 'odom', 10)
        self.create_timer(.1, self.publish_pose)
        self.server = ActionServer(self, NavigateToPose, 'navigate_to_pose',
            execute_callback=self.execute, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())

    def publish_pose(self):
        msg = Odometry()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = 3.4, 9.5
        msg.pose.pose.orientation.w = 1.
        self.odom.publish(msg)

    def execute(self, goal):
        self.get_logger().info('MOCK received navigation goal (interface test only)')
        end = time.monotonic() + 12
        while time.monotonic() < end:
            if goal.is_cancel_requested:
                goal.canceled()
                return NavigateToPose.Result()
            feedback = NavigateToPose.Feedback()
            feedback.distance_remaining = 1.
            goal.publish_feedback(feedback)
            time.sleep(.1)
        if self.get_parameter('outcome').value == 'abort':
            goal.abort()
        else:
            goal.succeed()
        return NavigateToPose.Result()


rclpy.init()
node = Fixture()
executor = MultiThreadedExecutor()
executor.add_node(node)
try:
    executor.spin()
except KeyboardInterrupt:
    pass
finally:
    executor.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

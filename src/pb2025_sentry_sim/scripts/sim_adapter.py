#!/usr/bin/env python3
"""Reusable Gazebo sensor, localization and chassis interface for Nav2."""
import copy
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, JointState
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rmoss_interfaces.msg import ChassisCmd


class SimAdapter(Node):
    def __init__(self):
        super().__init__('sentry_sim_adapter')
        robot = self.declare_parameter('robot_name', 'red_standard_robot1').value
        self.timeout = self.declare_parameter('command_timeout', 0.5).value
        self.pose = None
        self.last_odom = self.last_command = 0.
        self.yaw_joint = 0.
        self.command = Twist()
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        identity = TransformStamped()
        identity.header.frame_id = 'map'
        identity.child_frame_id = 'odom'
        identity.transform.rotation.w = 1.
        self.static_tf.sendTransform(identity)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(ChassisCmd, f'/{robot}/robot_base/chassis_cmd', 10)
        self.create_subscription(Odometry, f'/{robot}/chassis_odometry_gt', self.on_odom, 10)
        self.create_subscription(LaserScan, f'/{robot}/rplidar_a2/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(JointState, f'/{robot}/joint_states', self.on_joints, 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_command, 10)
        self.create_timer(0.05, self.control, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_odom(self, msg):
        # MecanumDrive2 publishes WorldPose even though its frame is named odom.
        msg = copy.deepcopy(msg)
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'chassis'
        self.pose = msg.pose.pose
        q = self.pose.orientation
        yaw = math.atan2(2. * (q.w * q.z + q.x * q.y), 1. - 2. * (q.y * q.y + q.z * q.z))
        # Upstream publishes world-frame linear velocity; ROS Odometry twist is
        # expressed in child_frame_id (chassis).
        vx, vy = msg.twist.twist.linear.x, msg.twist.twist.linear.y
        msg.twist.twist.linear.x = math.cos(yaw) * vx + math.sin(yaw) * vy
        msg.twist.twist.linear.y = -math.sin(yaw) * vx + math.cos(yaw) * vy
        self.last_odom = time.monotonic()
        self.odom_pub.publish(msg)
        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = 'chassis'
        transform.transform.translation.x = self.pose.position.x
        transform.transform.translation.y = self.pose.position.y
        transform.transform.translation.z = self.pose.position.z
        transform.transform.rotation = self.pose.orientation
        self.tf.sendTransform(transform)

    def on_scan(self, msg):
        msg.header.frame_id = 'front_rplidar_a2'
        self.scan_pub.publish(msg)

    def on_joints(self, msg):
        if 'gimbal_yaw_joint' in msg.name:
            self.yaw_joint = msg.position[msg.name.index('gimbal_yaw_joint')]

    def on_command(self, msg):
        self.command = msg
        self.last_command = time.monotonic()

    def control(self):
        now = time.monotonic()
        safe = now - self.last_odom < self.timeout and now - self.last_command < self.timeout
        msg = ChassisCmd()
        msg.type = ChassisCmd.VELOCITY
        if safe:
            # Pinned upstream rotates VELOCITY input by +gimbal yaw despite the
            # message contract. Pre-rotate to keep Nav2 commands in chassis axes.
            c, s = math.cos(self.yaw_joint), math.sin(self.yaw_joint)
            msg.twist.linear.x = c * self.command.linear.x + s * self.command.linear.y
            msg.twist.linear.y = -s * self.command.linear.x + c * self.command.linear.y
            msg.twist.angular.z = self.command.angular.z
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = SimAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(ChassisCmd(type=ChassisCmd.VELOCITY))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

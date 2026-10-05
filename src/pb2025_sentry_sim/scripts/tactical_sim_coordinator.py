#!/usr/bin/env python3
"""Optional tactical demo: command gate, referee truth injection and RViz display."""
import copy
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from pb_rm_interfaces.msg import RobotStatus
from pb2025_sentry_decision_interfaces.msg import TacticalDecision, TacticalExecution
from btcpp_ros2_interfaces.action import ExecuteTree
from nav2_msgs.action import NavigateToPose


class TacticalSimCoordinator(Node):
    def __init__(self):
        super().__init__('tactical_sim_coordinator')
        self.require_inputs = self.declare_parameter('require_navigation_inputs', True).value
        self.autonomy = self.declare_parameter('autonomy', True).value
        self.timeout = self.declare_parameter('command_timeout', 0.5).value
        self.pose = None
        self.decision = None
        self.execution = None
        self.goal = None
        self.last_odom = self.last_command = 0.
        self.last_decision = 0.
        self.stop_until = 0.
        self.command = Twist()
        self.scan_ready = False
        self.map = None
        self.tree_sent = False
        self.status_pub = self.create_publisher(RobotStatus, 'referee/robot_status', 10)
        self.cmd_pub = self.create_publisher(Twist, 'sim/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'tactical_markers', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(RobotStatus, 'sim_input/robot_status', self.on_status, 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_command, 10)
        self.create_subscription(Twist, 'safety_stop', self.on_stop, 10)
        self.create_subscription(TacticalDecision, '/tactical_decision', self.on_decision, 10)
        self.create_subscription(TacticalExecution, '/tactical_execution', self.on_execution, 10)
        self.create_subscription(PoseStamped, '/tactical_goal', self.on_goal, 10)
        self.create_subscription(OccupancyGrid, 'map', self.on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tree_client = ActionClient(self, ExecuteTree, 'pb2025_sentry_behavior')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_timer(0.05, self.control, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(0.2, self.display)

    def on_map(self, msg):
        self.map = msg

    def on_odom(self, msg):
        self.pose = msg.pose.pose
        self.last_odom = time.monotonic()

    def on_scan(self, msg):
        self.scan_ready = True

    def on_status(self, msg):
        if self.pose is not None:
            msg.robot_pos = copy.deepcopy(self.pose)
            self.status_pub.publish(msg)

    def on_command(self, msg):
        self.command = msg
        self.last_command = time.monotonic()

    def on_stop(self, msg):
        self.stop_until = time.monotonic() + 0.5
        self.command = Twist()

    def on_decision(self, msg):
        self.decision = msg
        self.last_decision = time.monotonic()

    def on_execution(self, msg):
        if msg.decision_id:
            self.execution = msg

    def on_goal(self, msg):
        self.goal = msg

    def control(self):
        now = time.monotonic()
        safe = now - self.last_odom < self.timeout and now - self.last_command < self.timeout and now >= self.stop_until
        if self.autonomy:
            safe = safe and self.decision is not None and now - self.last_decision < 1.0 and self.decision.action != TacticalDecision.HOLD_SAFE
        self.cmd_pub.publish(self.command if safe else Twist())
        if (self.autonomy and not self.tree_sent and (not self.require_inputs or (self.pose is not None and self.scan_ready and self.map is not None)) and self.nav_client.server_is_ready() and self.tree_client.server_is_ready()):
            self.tree_sent = True
            goal = ExecuteTree.Goal()
            goal.target_tree = 'sentry_tactical'
            self.tree_client.send_goal_async(goal).add_done_callback(self.tree_response)

    def tree_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Behavior tree goal rejected')
            return
        self.get_logger().info('Gazebo inputs and Nav2 ready; sentry_tactical accepted')

    def display(self):
        if self.pose is None:
            return
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'decision'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose = copy.deepcopy(self.pose)
        marker.pose.position.z += 1.5
        marker.scale.z = 0.35
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.
        decision = self.decision
        execution = self.execution
        names = ['HOLD_SAFE', 'COVER_TEAMMATE', 'ATTACK_OUTPOST', 'RESUPPLY_HOME', 'DEFEND_HOME', 'CAPTURE_POINT', 'GATHER_INFO']
        marker.text = 'SIMULATED REFEREE INPUT / 模拟输入\n'
        marker.text += f'RECOMMENDED: {decision.action_name} score={decision.score:.2f}\n' if decision else 'RECOMMENDED: waiting\n'
        if decision:
            marker.text += ', '.join(decision.top_reasons) + '\n'
        marker.text += (f"{'EXECUTING' if execution.state in ('STARTED', 'RUNNING') else 'LAST TASK'}: {names[execution.action]} {execution.state}\ndecision={execution.decision_id} world={execution.world_state_id}" if execution and execution.action < len(names) else 'EXECUTING: no task feedback')
        markers = [marker]
        if self.goal:
            target = Marker()
            target.header = self.goal.header
            target.header.stamp = marker.header.stamp
            target.ns = 'navigation_goal'
            target.id = 1
            target.type = Marker.ARROW
            target.pose = self.goal.pose
            target.scale.x, target.scale.y, target.scale.z = 0.8, 0.15, 0.15
            target.color.r, target.color.g, target.color.a = 1., 0.6, 1.
            markers.append(target)
        self.marker_pub.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = TacticalSimCoordinator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

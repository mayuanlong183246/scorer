# Copyright 2026 RMUC contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Exercise the actual BT server, parameter loading, and ROS subscriptions."""

import argparse
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.task import Future
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy, QoSProfile
from btcpp_ros2_interfaces.action import ExecuteTree
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from pb2025_sentry_decision_interfaces.msg import TacticalDecision, TacticalExecution
from pb_rm_interfaces.msg import GameStatus, RobotStatus, EventData
from std_msgs.msg import Bool


def main():
    """Verify a real server can construct, execute, and halt the configured tree."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--server', required=True)
    parser.add_argument('--params', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('tactical_server_test')
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    mapping = node.create_publisher(Bool, '/mapping_bootstrap_finished', latched)
    mapping.publish(Bool(data=True))
    games = node.create_publisher(GameStatus, '/referee/game_status', 10)
    robots = node.create_publisher(RobotStatus, '/referee/robot_status', 10)
    referee_events = node.create_publisher(EventData, '/referee/event_data', 10)
    decisions = node.create_publisher(TacticalDecision, '/tactical_decision', 10)
    locks = node.create_publisher(Bool, '/auto_aim/target_locked', 10)
    events = []
    stops = []
    node.create_subscription(TacticalExecution, '/tactical_execution', events.append, 10)
    node.create_subscription(Twist, '/cmd_vel', stops.append, 10)

    hold_navigation = False
    pending_navigation = []
    canceled_navigation = []

    async def navigate(handle):
        if hold_navigation:
            done = Future()
            pending_navigation.append((handle, done))
            await done
            handle.canceled()
            canceled_navigation.append(handle)
        else:
            handle.succeed()
        return NavigateToPose.Result()

    navigation = ActionServer(node, NavigateToPose, '/navigate_to_pose', navigate,
                              cancel_callback=lambda _: CancelResponse.ACCEPT,
                              callback_group=ReentrantCallbackGroup())
    with tempfile.TemporaryFile(mode='w+') as log:
        proc = subprocess.Popen([
            args.server, '--ros-args', '-r', '__node:=pb2025_sentry_behavior_server',
            '--params-file', args.params, '-p', 'use_sim_time:=false',
            '-p', 'mapping_bootstrap_enabled:=true', '-p', 'groot2_port:=1769',
            '-p', 'tick_frequency:=50', '-p', 'gather_info_dwell_ms:=200',
        ], stdout=log, stderr=log)
        try:
            client = ActionClient(node, ExecuteTree, '/pb2025_sentry_behavior')
            assert client.wait_for_server(timeout_sec=8), 'BT server unavailable'
            game = GameStatus(game_progress=4, stage_remain_time=400)
            decision = TacticalDecision(action=0, decision_id=7001)
            lock = Bool(data=True)

            def pump(seconds):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    games.publish(game)
                    robots.publish(RobotStatus(current_hp=400, projectile_allowance_17mm=150))
                    referee_events.publish(EventData(base_hp=10000))
                    decisions.publish(decision)
                    locks.publish(lock)
                    rclpy.spin_once(node, timeout_sec=0.01)
                    for nav_handle, done in pending_navigation:
                        if nav_handle.is_cancel_requested and not done.done():
                            done.set_result(True)

            pump(0.4)
            future = client.send_goal_async(ExecuteTree.Goal(target_tree='sentry_tactical'))
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            handle = future.result()
            assert handle and handle.accepted, 'tree rejected'
            result = handle.get_result_async()
            pump(0.5)
            assert any(e.action == 0 and e.state == 'STARTED' and e.decision_id == 7001
                       for e in events), events
            assert not stops, 'decision must not publish velocity'
            decision.action = 6
            decision.decision_id = 7002
            pump(0.6)
            assert any(e.action == 6 and e.state == 'STARTED' for e in events), events
            assert not any(e.action == 6 and e.state == 'SUCCEEDED' for e in events), events
            lock.data = False
            pump(0.5)
            assert any(e.action == 6 and e.state == 'SUCCEEDED' for e in events), events
            decision.action = 0
            decision.decision_id = 7003
            pump(0.2)
            game.game_progress = 5
            pump(0.2)
            assert any(e.state == 'PREEMPTED' and e.reason == 'game_not_running'
                       for e in events), events
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=3)
            rclpy.spin_until_future_complete(node, result, timeout_sec=3)
            assert result.done(), 'tree did not stop'
            # Halt a tree while navigation is active: cancellation must complete
            # even though the BT dispatcher is no longer polling its transport.
            game.game_progress = 4
            decision.action = 6
            decision.decision_id = 7004
            hold_navigation = True
            pump(0.1)
            future = client.send_goal_async(ExecuteTree.Goal(target_tree='sentry_tactical'))
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
            active_tree = future.result()
            assert active_tree and active_tree.accepted
            pump(0.4)
            assert pending_navigation, 'navigation never started'
            halted = active_tree.cancel_goal_async()
            rclpy.spin_until_future_complete(node, halted, timeout_sec=3)
            pump(0.5)
            assert canceled_navigation, 'navigation cancellation stopped with the tree'
            assert not stops, 'decision must not publish velocity'
            print('PASS: server parameters, plugins, latched mapping, lock status, '
                  'whole-action feedback and game end')
        except BaseException:
            log.seek(0)
            print(log.read())
            raise
        finally:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
            navigation.destroy()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

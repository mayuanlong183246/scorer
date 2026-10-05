"""No DDS/Gazebo required: verify transforms and fail-closed velocity forwarding."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

spec = importlib.util.spec_from_file_location('adapter', Path(__file__).parents[1] / 'scripts/sim_adapter.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Publisher:
    def publish(self, msg):
        self.message = msg

    sendTransform = publish


class AdapterTest(unittest.TestCase):
    def backend(self):
        return SimpleNamespace(timeout=.5, last_odom=10., last_command=10.,
                               stop_until=0., yaw_joint=0., command=Twist(),
                               cmd_pub=Publisher(), odom_pub=Publisher(), tf=Publisher())

    def test_velocity_and_watchdog(self):
        node = self.backend()
        node.command.linear.x = 1.
        node.yaw_joint = math.pi / 2
        with patch.object(module.time, 'monotonic', return_value=10.1):
            module.SimAdapter.control(node)
        self.assertAlmostEqual(node.cmd_pub.message.twist.linear.y, -1.)
        for field in ('last_odom', 'last_command'):
            node.last_odom = node.last_command = 10.
            setattr(node, field, 9.)
            with patch.object(module.time, 'monotonic', return_value=10.1):
                module.SimAdapter.control(node)
            self.assertEqual(node.cmd_pub.message.twist, Twist())

    def test_world_velocity_to_chassis(self):
        node = self.backend()
        msg = Odometry()
        msg.pose.pose.position.x = 4.
        msg.pose.pose.orientation.z = math.sin(math.pi / 4)
        msg.pose.pose.orientation.w = math.cos(math.pi / 4)
        msg.twist.twist.linear.x = 1.
        module.SimAdapter.on_odom(node, msg)
        self.assertAlmostEqual(node.odom_pub.message.twist.twist.linear.y, -1.)
        self.assertEqual(node.tf.message.child_frame_id, 'chassis')
        self.assertEqual(node.tf.message.transform.translation.x, 4.)
        self.assertEqual(msg.twist.twist.linear.y, 0.)


if __name__ == '__main__':
    unittest.main()

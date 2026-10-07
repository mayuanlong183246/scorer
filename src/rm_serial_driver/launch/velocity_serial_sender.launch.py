import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("rm_serial_driver"), "config", "serial_driver.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=config),
        DeclareLaunchArgument("device_name", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("baud_rate", default_value="115200"),
        DeclareLaunchArgument("flow_control", default_value="none"),
        DeclareLaunchArgument("parity", default_value="none"),
        DeclareLaunchArgument("stop_bits", default_value="1"),
        DeclareLaunchArgument("dry_run", default_value="false"),
        Node(
            package="rm_serial_driver",
            executable="velocity_serial_sender_node",
            name="velocity_serial_sender",
            output="screen",
            parameters=[
                config,
                {
                    "device_name": LaunchConfiguration("device_name"),
                    "baud_rate": ParameterValue(LaunchConfiguration("baud_rate"), value_type=int),
                    "flow_control": LaunchConfiguration("flow_control"),
                    "parity": LaunchConfiguration("parity"),
                    "stop_bits": LaunchConfiguration("stop_bits"),
                    "dry_run": ParameterValue(LaunchConfiguration("dry_run"), value_type=bool),
                },
            ],
        ),
    ])

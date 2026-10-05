"""Reusable Gazebo + Nav2 environment; no tactical nodes or inputs required."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    pkg = share('pb2025_sentry_sim')
    def include(file, names):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', file)),
            launch_arguments={name: LaunchConfiguration(name) for name in names}.items())
    return [include('gazebo_backend.launch.py', ['gui', 'robot_name', 'cmd_vel_topic']),
            include('nav2.launch.py', ['rviz', 'map', 'params_file'])]


def generate_launch_description():
    pkg = share('pb2025_sentry_sim')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('robot_name', default_value='red_standard_robot1'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config/nav2.yaml')),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps/rmuc_2025.yaml')),
        OpaqueFunction(function=setup),
    ])

"""Single red robot, RMUC 2025, truth localization and actual Nav2 actions."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    pkg = share('pb2025_sentry_sim')
    robot = LaunchConfiguration('robot_name').perform(context)
    autonomy = LaunchConfiguration('autonomy').perform(context).lower() == 'true'
    map_file = LaunchConfiguration('map').perform(context)
    nodes = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch/gazebo_nav2.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'), 'rviz': LaunchConfiguration('rviz'),
            'robot_name': robot, 'map': map_file,
            'cmd_vel_topic': 'sim/cmd_vel' if autonomy else 'cmd_vel',
        }.items())]
    if autonomy:
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch/decision_test.launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in
                              ['scenario', 'groot2_port', 'map', 'log_file']}.items()))
    return nodes


def generate_launch_description():
    pkg = share('pb2025_sentry_sim')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('groot2_port', default_value='1768'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('autonomy', default_value='true'),
        DeclareLaunchArgument('robot_name', default_value='red_standard_robot1'),
        DeclareLaunchArgument('scenario', default_value='hold_safe'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps/rmuc_2025.yaml')),
        DeclareLaunchArgument('log_file', default_value='auto'),
        OpaqueFunction(function=setup),
    ])

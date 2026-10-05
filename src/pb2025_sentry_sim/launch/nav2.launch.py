"""Standard Nav2 and map server; accepts sensors from any compatible backend."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    pkg = share('pb2025_sentry_sim')
    params = LaunchConfiguration('params_file').perform(context)
    map_file = LaunchConfiguration('map').perform(context)
    common = {'use_sim_time': True}
    nodes = [
        Node(package='nav2_map_server', executable='map_server', name='map_server', parameters=[params, {'yaml_filename': map_file}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='map_lifecycle', parameters=[common, {'autostart': True, 'node_names': ['map_server']}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share('nav2_bringup'), 'launch/navigation_launch.py')),
            launch_arguments={'use_sim_time': 'true', 'params_file': params, 'autostart': 'true', 'use_composition': 'False'}.items()),
    ]
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2', parameters=[common], arguments=['-d', os.path.join(pkg, 'config/decision.rviz')]))
    return nodes


def generate_launch_description():
    pkg = share('pb2025_sentry_sim')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config/nav2.yaml')),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps/rmuc_2025.yaml')),
        OpaqueFunction(function=setup),
    ])

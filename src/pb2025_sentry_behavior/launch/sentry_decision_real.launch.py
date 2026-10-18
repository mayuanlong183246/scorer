# Copyright 2026 RMUC contributors
# Licensed under the Apache License, Version 2.0.
"""Decision-only real robot launch; navigation owns motion and cancellation stops."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    defaults = {
        'namespace': '',
        'params_file': os.path.join(get_package_share_directory('pb2025_sentry_behavior'),
                                    'params', 'sentry_behavior.yaml'),
        'navigation_action_name': 'navigate_to_pose',
        'team_color': 'blue',
    }
    namespace = LaunchConfiguration('namespace')
    params = ParameterFile(RewrittenYaml(
        source_file=LaunchConfiguration('params_file'), root_key=namespace,
        param_rewrites={'use_sim_time': 'false', 'tick_frequency': '20', 'tick_hz': '5.0',
                        'navigation_timeout_ms': '180000',
                        'navigation_cancel_timeout_ms': '2000',
                        'target_tree': 'sentry_tactical',
                        'navigation_action_name': LaunchConfiguration('navigation_action_name'),
                        'team_color': LaunchConfiguration('team_color')},
        convert_types=True), allow_substs=True)
    nodes = [Node(package='pb2025_sentry_behavior', executable=executable, name=executable,
                  output='screen', parameters=[params] + (
                      [{'validate_navigation_goals': True}]
                      if executable == 'pb2025_sentry_behavior_server' else []))
             for executable in ('tactical_scorer_node', 'pb2025_sentry_behavior_server',
                                'pb2025_sentry_behavior_client')]
    return LaunchDescription(
        [DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()] +
        [PushRosNamespace(namespace)] + nodes)

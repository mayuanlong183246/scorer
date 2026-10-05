# Copyright 2026 RMUC contributors
# Licensed under the Apache License, Version 2.0.
"""Existing physical navigation with isolated fake referee and an opt-in output gate."""
import math
import os
from pathlib import Path
import tempfile

from PIL import Image
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def validate_goals(map_file, goals):
    metadata = yaml.safe_load(Path(map_file).read_text())
    pixels = Image.open(Path(map_file).parent / metadata['image']).convert('L')
    ox, oy, yaw = metadata['origin']
    for name, value in goals.items():
        if not name.endswith('_goal'):
            continue
        parts = [float(v) for v in value.split(';')]
        if len(parts) not in (3, 7) or not all(math.isfinite(v) for v in parts):
            raise ValueError(f'{name}: invalid pose {value}')
        dx, dy = parts[0] - ox, parts[1] - oy
        ix = math.floor((math.cos(yaw) * dx + math.sin(yaw) * dy) / metadata['resolution'])
        iy = math.floor((-math.sin(yaw) * dx + math.cos(yaw) * dy) / metadata['resolution'])
        if not (0 <= ix < pixels.width and 0 <= iy < pixels.height):
            raise ValueError(f'{name}: outside map: {value}')
        probability = pixels.getpixel((ix, pixels.height - 1 - iy)) / 255.
        if not metadata.get('negate', 0):
            probability = 1. - probability
        if probability >= metadata['free_thresh']:
            raise ValueError(f'{name}: occupied or unknown map cell: {value}')


def setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    behavior = yaml.safe_load(Path(arg('params_file')).read_text())
    navigation = yaml.safe_load(Path(arg('nav_params_file')).read_text())
    server = behavior['pb2025_sentry_behavior_server']['ros__parameters']
    validate_goals(arg('map'), server)
    for block in behavior.values():
        block['ros__parameters']['use_sim_time'] = False
        block['ros__parameters']['team_color'] = arg('team_color')
    server['validate_navigation_goals'] = True
    # Poll cancellation and stop requests fast enough for the 0.2 s test target.
    server['tick_frequency'] = 20
    # Physical motion takes longer than the short Gazebo demo.
    server['navigation_timeout_ms'] = 180000
    for block in navigation.values():
        if isinstance(block, dict) and 'ros__parameters' in block:
            block['ros__parameters']['use_sim_time'] = False
    transform = navigation['fake_vel_transform']['ros__parameters']
    transform.update(output_cmd_vel_topic='/test_velocity/converted',
                     cmd_spin_topic='/test_disabled_spin', init_spin_speed=0.0)
    smoother = navigation['velocity_smoother']['ros__parameters']
    smoother.update(max_velocity=[0.3, 0.3, 0.5], min_velocity=[-0.3, -0.3, -0.5])
    controller = navigation['controller_server']['ros__parameters']['FollowPath']
    controller.update(vx_max=0.3, vx_min=-0.3, vy_max=0.3, wz_max=0.5)
    navigation['behavior_server']['ros__parameters'].update(max_rotational_vel=0.5,
                                                         min_rotational_vel=0.1)
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='sentry_real_test_',
                                     delete=False) as output:
        yaml.safe_dump(navigation, output)
        nav_file = output.name
    topics = ['game_status', 'robot_status', 'event_data', 'rfid_status',
              'all_robot_hp', 'ground_robot_position', 'simulation_state', 'buff']
    remaps = [('/referee/' + topic, '/test_referee/' + topic) for topic in topics]
    nodes = []
    if arg('start_navigation').lower() == 'true':
        nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            share('pb2025_nav_bringup'), 'launch/rm_navigation_reality_launch.py')),
            launch_arguments={'map': arg('map'), 'params_file': nav_file,
                              'use_sim_time': 'false', 'use_rviz': arg('rviz')}.items()))
    nodes.append(Node(package='pb2025_sentry_behavior', executable='sentry_referee_simulator.py',
                      name='sentry_referee_simulator', output='screen', remappings=remaps,
                      parameters=[{'use_sim_time': False, 'team_color': arg('team_color'),
                                   'scenario': 'hold_safe', 'publish_target': False}]))
    for executable, name in [('tactical_scorer_node', 'tactical_scorer_node'),
                             ('pb2025_sentry_behavior_server', 'pb2025_sentry_behavior_server'),
                             ('pb2025_sentry_behavior_client', 'pb2025_sentry_behavior_client')]:
        params = behavior.get(name, {}).get('ros__parameters', {})
        nodes.append(Node(package='pb2025_sentry_behavior', executable=executable,
                          name=name, output='screen', parameters=[params],
                          remappings=remaps + [('/cmd_vel', '/safety_stop')]))
    nodes.extend([
        Node(package='pb2025_sentry_behavior', executable='sentry_tactical_visualizer.py',
             output='screen', remappings=remaps,
             parameters=[{'use_sim_time': False, 'team_color': arg('team_color'),
                          'log_file': arg('log_file')}]),
        Node(package='pb2025_sentry_behavior', executable='sentry_test_velocity_gate.py',
             output='screen', parameters=[{'enabled': LaunchConfiguration('gate_enabled'), 'output_topic': '/cmd_vel_base_real_yaw'}]),
    ])
    return nodes


def generate_launch_description():
    nav = share('pb2025_nav_bringup')
    behavior = share('pb2025_sentry_behavior')
    defaults = {'map': os.path.join(nav, 'map/RMUC26.yaml'),
                'nav_params_file': os.path.join(nav, 'config/reality/nav2_params.yaml'),
                'params_file': os.path.join(behavior, 'params/sentry_behavior.yaml'),
                'start_navigation': 'true', 'rviz': 'true', 'team_color': 'blue', 'gate_enabled': 'false',
                'log_file': 'auto', 'gate_enabled': 'false'}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v)
                              for k, v in defaults.items()] + [OpaqueFunction(function=setup)])

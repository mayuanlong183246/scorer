#!/usr/bin/env python3
"""Standalone referee simulator publishing the standard /referee topics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='match'),
        DeclareLaunchArgument('team_color', default_value='blue'),
        DeclareLaunchArgument('publish_target', default_value='true'),
        DeclareLaunchArgument('publish_rate', default_value='5.0'),
        DeclareLaunchArgument('scenario_period_sec', default_value='6.0'),
        DeclareLaunchArgument('match_duration_sec', default_value='180.0'),
        DeclareLaunchArgument('speed', default_value='1.0'),
        Node(
            package='pb2025_sentry_behavior',
            executable='sentry_referee_simulator.py',
            name='sentry_referee_simulator',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'scenario': LaunchConfiguration('scenario'),
                'team_color': LaunchConfiguration('team_color'),
                'publish_target': LaunchConfiguration('publish_target'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'scenario_period_sec': LaunchConfiguration('scenario_period_sec'),
                'match_duration_sec': LaunchConfiguration('match_duration_sec'),
                'speed': LaunchConfiguration('speed'),
            }],
        ),
    ])

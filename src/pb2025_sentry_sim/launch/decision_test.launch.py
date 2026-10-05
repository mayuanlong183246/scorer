"""Decision test inputs and observers; navigation is provided by an external action server."""
import os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    pkg = share('pb2025_sentry_sim')
    behavior = os.path.join(share('pb2025_sentry_behavior'), 'params/sentry_behavior.yaml')
    tactical = os.path.join(pkg, 'config/tactical.yaml')
    map_file = LaunchConfiguration('map').perform(context)
    # Reject map/goal mismatch before starting any processes.
    from PIL import Image
    import numpy as np
    from scipy.ndimage import distance_transform_edt, label
    metadata = yaml.safe_load(Path(map_file).read_text())
    if metadata['origin'][2] != 0.0:
        raise RuntimeError('Simulation map must use the Gazebo world axes (zero origin yaw)')
    pixels = np.array(Image.open(Path(map_file).parent / metadata['image']))[::-1]
    free = pixels > 250
    clearance = distance_transform_edt(free) * metadata['resolution']
    regions, _ = label(clearance > 0.4)
    def cell(x, y):
        return int((y - metadata['origin'][1]) / metadata['resolution']), int((x - metadata['origin'][0]) / metadata['resolution'])
    sy, sx = cell(3.4, 9.5)
    if not (0 <= sy < regions.shape[0] and 0 <= sx < regions.shape[1]):
        raise RuntimeError('Test spawn is outside the navigation map')
    region = regions[sy, sx]
    goals = yaml.safe_load(Path(tactical).read_text())['pb2025_sentry_behavior_server']['ros__parameters']
    for key, value in goals.items():
        if key.endswith('_goal'):
            x, y, _ = map(float, value.split(';'))
            iy, ix = cell(x, y)
            if not (0 <= iy < regions.shape[0] and 0 <= ix < regions.shape[1]) or region == 0 or regions[iy, ix] != region:
                raise RuntimeError(f'{key} is blocked or disconnected from spawn: {value}')
    common = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    nodes = []
    nodes += [
        Node(package='pb2025_sentry_sim', executable='tactical_sim_coordinator.py', parameters=[common, {'require_navigation_inputs': ParameterValue(LaunchConfiguration('require_navigation_inputs'), value_type=bool)}]),
        Node(package='pb2025_sentry_behavior', executable='sentry_referee_simulator.py', name='sentry_referee_simulator', parameters=[common, {'scenario': LaunchConfiguration('scenario').perform(context), 'team_color': 'red', 'gazebo_navigation': True}], remappings=[('referee/robot_status', 'sim_input/robot_status')]),
        Node(package='pb2025_sentry_behavior', executable='tactical_scorer_node', name='tactical_scorer_node', parameters=[behavior, common, {'team_color': 'red'}]),
        Node(package='pb2025_sentry_behavior', executable='pb2025_sentry_behavior_server', name='pb2025_sentry_behavior_server', parameters=[behavior, tactical, common, {'groot2_port': ParameterValue(LaunchConfiguration('groot2_port'), value_type=int)}], remappings=[('/cmd_vel', '/safety_stop')]),
        Node(package='pb2025_sentry_behavior', executable='sentry_tactical_visualizer.py', parameters=[common, {'team_color': 'red', 'log_file': LaunchConfiguration('log_file').perform(context)}]),
    ]
    return nodes


def generate_launch_description():
    pkg = share('pb2025_sentry_sim')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('require_navigation_inputs', default_value='true'),
        DeclareLaunchArgument('groot2_port', default_value='1768'),
        DeclareLaunchArgument('scenario', default_value='hold_safe'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps/rmuc_2025.yaml')),
        DeclareLaunchArgument('log_file', default_value='auto'),
        OpaqueFunction(function=setup),
    ])

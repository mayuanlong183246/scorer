"""Reusable Gazebo + Nav2 environment; no tactical nodes or inputs required."""
import os
from pathlib import Path
import yaml
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def setup(context):
    from sdformat_tools.urdf_generator import UrdfGenerator
    from xmacro.xmacro4sdf import XMLMacro4sdf
    pkg = share('pb2025_sentry_sim')
    upstream = share('rmu_gazebo_simulator')
    robot = LaunchConfiguration('robot_name').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    xmacro = XMLMacro4sdf()
    xmacro.set_xml_file(os.path.join(share('pb2025_robot_description'), 'resource/xmacro/simulation_robot.sdf.xmacro'))
    xmacro.generate({'global_initial_color': 'red'})
    model_xml = ET.fromstring(xmacro.to_string())
    # Navigation needs the 2D lidar, not cameras or combat visual plugins.
    # Fortress LightBarController duplicates rendering visuals in headless mode.
    for parent in model_xml.iter():
        for child in list(parent):
            if (child.tag == 'plugin' and child.get('filename') in ('LightBarController', 'ProjectileShooter')) or (child.tag == 'sensor' and child.get('type') == 'camera'):
                parent.remove(child)
    sdf = ET.tostring(model_xml, encoding='unicode')
    urdf = UrdfGenerator()
    urdf.parse_from_sdf_string(sdf)
    world = LaunchConfiguration('world').perform(context)
    world_file = Path(upstream) / 'resource/worlds' / (world + '_world.sdf')
    mesh = Path(upstream) / 'resource/models' / world / 'model.sdf'
    if not world_file.is_file() or not mesh.is_file():
        raise RuntimeError(f'Missing upstream world/model: {world}')
    config = yaml.safe_load((Path(upstream) / 'config/gz_world.yaml').read_text())
    spawn = config['robots'][world][0]
    resource_paths = os.pathsep.join([str(Path(upstream) / 'resource/models'), str(Path(upstream) / 'resource/worlds'), os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')])
    common = {'use_sim_time': True}
    nodes = [
        # Gazebo transport does not use ROS_DOMAIN_ID. Give each launch its own
        # world/clock unless a caller explicitly requests a shared partition.
        SetEnvironmentVariable("IGN_PARTITION", os.environ.get(
            "IGN_PARTITION", f"sentry_{os.environ.get('ROS_DOMAIN_ID', '0')}_{os.getpid()}")),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", resource_paths),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share('ros_gz_sim'), 'launch/gz_sim.launch.py')),
            launch_arguments={'gz_version': '6', 'gz_args': str(world_file) + (' -r --gui-config ' + os.path.join(upstream, 'resource/ign/gui.config') if gui else ' -r -s --headless-rendering')}.items()),
        Node(package='ros_gz_sim', executable='create', arguments=['-string', sdf, '-name', robot, '-allow_renaming', 'false', '-x', str(spawn['x_pose']), '-y', str(spawn['y_pose']), '-z', str(spawn['z_pose']), '-Y', str(spawn['yaw'])]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=['/world/default/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'], remappings=[('/world/default/clock', '/clock')]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='robot_bridge', parameters=[{'config_file': ReplaceString(source_file=os.path.join(upstream, 'config/ros_gz_bridge.yaml'), replacements={'<robot_name>': robot})}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[common, {'robot_description': urdf.to_string()}], remappings=[('joint_states', f'/{robot}/joint_states')]),
        Node(package='rmoss_gz_base', executable='rmua19_robot_base', namespace=robot, parameters=[os.path.join(upstream, 'config/base_params.yaml'), common, {'robot_name': robot}]),
        Node(package='pb2025_sentry_sim', executable='sim_adapter.py', parameters=[common, {'robot_name': robot, 'command_timeout': 0.5}], remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_topic'))]),
    ]
    return nodes


def generate_launch_description():
    pkg = share('pb2025_sentry_sim')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('world', default_value='rmuc_2025', choices=['rmul_2024', 'rmuc_2024', 'rmul_2025', 'rmuc_2025']),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel'),
        DeclareLaunchArgument('robot_name', default_value='red_standard_robot1'),
        OpaqueFunction(function=setup),
    ])

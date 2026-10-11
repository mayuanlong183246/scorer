import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rm_serial_driver'), 'config', 'serial_driver.yaml')

    config_file = LaunchConfiguration('config_file')
    device_name = LaunchConfiguration('device_name')
    baud_rate = LaunchConfiguration('baud_rate')
    flow_control = LaunchConfiguration('flow_control')
    parity = LaunchConfiguration('parity')
    stop_bits = LaunchConfiguration('stop_bits')
    dry_run = LaunchConfiguration('dry_run')
    input_topic = LaunchConfiguration('input_topic')
    status_topic = LaunchConfiguration('status_topic')

    rm_serial_driver_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[
            config_file,
            {
                'device_name': device_name,
                'baud_rate': ParameterValue(baud_rate, value_type=int),
                'flow_control': ParameterValue(flow_control, value_type=str),
                'parity': ParameterValue(parity, value_type=str),
                'stop_bits': ParameterValue(stop_bits, value_type=str),
                'dry_run': ParameterValue(dry_run, value_type=bool),
                'input_topic': ParameterValue(input_topic, value_type=str),
                'status_topic': ParameterValue(status_topic, value_type=str),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config),
        DeclareLaunchArgument('device_name', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('flow_control', default_value='none'),
        DeclareLaunchArgument('parity', default_value='none'),
        DeclareLaunchArgument('stop_bits', default_value='1'),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('input_topic', default_value='/cmd_vel_base_real_yaw'),
        DeclareLaunchArgument('status_topic', default_value='/sentry_status'),
        rm_serial_driver_node,
    ])

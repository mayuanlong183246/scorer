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
        rm_serial_driver_node,
    ])

#5-6 14-20 27-36 39-47
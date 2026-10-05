import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    config = os.path.join(
        get_package_share_directory('fast_lio'), 'config', 'mid360.yaml')

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[
            config
        ],
        output='screen',
        remappings=[('/Odometry','/aft_mapped_to_init')]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    sensor_scan_generation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('sensor_scan_generation'),
                    'launch',
                    'sensor_scan_generation.launch.py',
                )
            )
        )
    loam_interface_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('loam_interface'),
                    'launch',
                    'loam_interface_launch.py',
                )
            )
        )
    livox_ros_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('livox_ros_driver2'),
                    'launch_ROS2',
                    'rviz_MID360_launch.py',
                )
            )
        )


    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    ld.add_action(fast_lio_node)
    ld.add_action(loam_interface_launch)
    ld.add_action(sensor_scan_generation_launch)
    ld.add_action(livox_ros_launch)
    # ld.add_action(rviz_node)

    return ld


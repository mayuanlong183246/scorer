import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch

################### user configure parameters for ros2 start ###################
xfer_format   = 1    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 1    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 20.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'right_lidar' # frame_id in published point cloud message, e.g., right_lidar, left_lidar, etc.
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
rviz_config_path = os.path.join(cur_config_path, 'display_point_cloud_ROS2.rviz')
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]


def generate_launch_description():
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
        )

    livox_rviz = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['--display-config', rviz_config_path]
        )

    # Static transform: base_footprint -> right_lidar (for lidar 151)
    left_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='left_lidar_broadcaster',
        arguments=[
            '--x', '-0.13', '--y', '0.13', '--z', '0.4',
            '--roll', '-0.6458', '--pitch', '0.9076', '--yaw', '1.0036',
            '--frame-id', 'base_footprint', '--child-frame-id', 'left_lidar'
        ]
    )

    # Static transform: base_footprint -> left_lidar (for lidar 159)
    right_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='right_lidar_broadcaster',
        arguments=[
            '--x', '-0.13', '--y', '-0.13', '--z', '0.4',
            '--roll', '0.6458', '--pitch', '0.9076', '--yaw', '-1.0036',
            '--frame-id', 'base_footprint', '--child-frame-id', 'right_lidar'
        ]
    )

    # FAST-LIO extrinsic is defined as T_imu_lidar, so publish the inverse here:
    # right_lidar -> imu_link
    right_lidar_to_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='right_lidar_to_imu_broadcaster',
        arguments=[
            '--x', '0.011', '--y', '0.02329', '--z', '-0.04412',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'right_lidar', '--child-frame-id', 'imu_link'
        ]
    )

    return LaunchDescription([
        livox_driver,
        #livox_rviz,
        right_lidar_tf,
        left_lidar_tf,
        right_lidar_to_imu_tf,
    ])

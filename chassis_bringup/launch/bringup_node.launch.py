import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('chassis_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    livox_share_dir = get_package_share_directory('livox_ros_driver2')

    user_config_path = os.path.join(
        livox_share_dir,
        'config',
        'MID360_config.json',
    )

    livox_ros2_params = [
        {'xfer_format': 1},  # 0-PointCloud2, 1-customized pointcloud format
        {'multi_topic': 0},  # 0-All LiDARs share the same topic
        {'data_src': 0},     # 0-lidar, others-Invalid data src
        {'publish_freq': 10.0},
        {'output_data_type': 0},
        {'frame_id': 'livox_frame'},
        {'lvx_file_path': '/home/livox/livox_test.lvx'},
        {'user_config_path': user_config_path},
        {'cmdline_input_bd_code': 'livox0000000001'},
    ]

    chassis_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'base_serial.launch.py')),
        launch_arguments={'use_wheeltec_imu': 'false'}.items(),
    )

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params,
    )

    livox_to_pointcloud2 = Node(
        package='livox_to_pointcloud2',
        executable='livox_to_pointcloud2_node',
        name='livox_to_pointcloud2',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(chassis_bringup)
    ld.add_action(livox_driver)
    ld.add_action(livox_to_pointcloud2)
    return ld

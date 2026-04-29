import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def make_static_tf_node(translation, rotation, parent, child, name):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            '--x', translation[0],
            '--y', translation[1],
            '--z', translation[2],
            '--roll', rotation[0],
            '--pitch', rotation[1],
            '--yaw', rotation[2],
            '--frame-id', parent,
            '--child-frame-id', child,
        ],
        output='screen',
    )


def generate_launch_description():
    use_intra_process_comms = LaunchConfiguration('use_intra_process_comms')

    bringup_dir = get_package_share_directory('chassis_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    livox_share_dir = get_package_share_directory('livox_ros_driver2')
    robot_urdf_path = os.path.join(
        get_package_share_directory('wheeltec_robot_urdf'),
        'urdf',
        'mini_mec_robot.urdf',
    )

    with open(robot_urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    user_config_path = os.path.join(
        livox_share_dir,
        'config',
        'MID360_config.json',
    )

    livox_ros2_params = [
        {'xfer_format': 1},
        {'multi_topic': 0},
        {'data_src': 0},
        {'publish_freq': 10.0},
        {'output_data_type': 0},
        {'frame_id': 'livox_frame'},
        {'lvx_file_path': '/home/livox/livox_test.lvx'},
        {'user_config_path': user_config_path},
        {'cmdline_input_bd_code': 'livox0000000001'},
    ]

    declare_use_intra_process_comms = DeclareLaunchArgument(
        'use_intra_process_comms',
        default_value='True',
        description='Enable ROS 2 intra-process communication for composable Livox nodes',
    )

    chassis_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'base_serial.launch.py')),
        launch_arguments={'use_wheeltec_imu': 'false'}.items(),
    )

    base_footprint_to_base_link_tf = make_static_tf_node(
        ['0.0', '0.0', '0.0'],
        ['0.0', '0.0', '0.0'],
        'base_footprint',
        'base_link',
        'base_footprint_to_base_link_tf',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    livox_container = ComposableNodeContainer(
        name='livox_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='livox_ros_driver2',
                plugin='livox_ros::DriverNode',
                name='livox_lidar_publisher',
                parameters=livox_ros2_params,
                extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
            ),
            ComposableNode(
                package='livox_to_pointcloud2',
                plugin='livox_to_pointcloud2::LivoxToPointCloud2',
                name='livox_to_pointcloud2',
                extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
            ),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_intra_process_comms)
    ld.add_action(chassis_bringup)
    ld.add_action(base_footprint_to_base_link_tf)
    ld.add_action(robot_state_publisher)
    ld.add_action(livox_container)
    return ld

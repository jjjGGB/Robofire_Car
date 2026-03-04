import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    # Get the launch directory
    robot_navigation_dir = get_package_share_directory('robot_navigation')
    navigation2_launch_dir = os.path.join(get_package_share_directory('rm_navigation'), 'launch')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_lio_rviz = LaunchConfiguration('lio_rviz')
    use_nav_rviz = LaunchConfiguration('nav_rviz')

    # ========================== Parameters Path ==========================
    # Ground segmentation parameters
    segmentation_params = os.path.join(
        robot_navigation_dir, 'config', 'simulation', 'segmentation_sim.yaml')

    # FAST_LIO parameters
    fastlio_mid360_params = os.path.join(
        robot_navigation_dir, 'config', 'simulation', 'fastlio_mid360_sim.yaml')
    fastlio_rviz_cfg_dir = os.path.join(
        robot_navigation_dir, 'rviz', 'fastlio.rviz')

    # Navigation2 parameters
    nav2_map_dir = os.path.join(robot_navigation_dir, 'map', 'grid_map.yaml')
    nav2_params_file_dir = os.path.join(
        robot_navigation_dir, 'config', 'simulation', 'nav2_params_sim.yaml')

    # base_link -> livox_frame transform (from measurement_params_sim.yaml)
    base_link2livox_xyz = [0.12, 0.0, 0.175]
    base_link2livox_rpy = [0.0, 0.0, 0.0]

    # ========================== Launch Arguments ==========================
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_use_lio_rviz_cmd = DeclareLaunchArgument(
        'lio_rviz',
        default_value='False',
        description='Visualize FAST_LIO cloud_map if true')

    declare_nav_rviz_cmd = DeclareLaunchArgument(
        'nav_rviz',
        default_value='True',
        description='Visualize navigation2 if true')

    # ========================== TF Static Transforms ==========================

    # Static TF: base_link -> livox_frame
    # FAST-LIO publishes odom -> base_link, this provides base_link -> livox_frame
    tf_base_link_to_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_livox_tf',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '--x', str(base_link2livox_xyz[0]),
            '--y', str(base_link2livox_xyz[1]),
            '--z', str(base_link2livox_xyz[2]),
            '--roll', str(base_link2livox_rpy[0]),
            '--pitch', str(base_link2livox_rpy[1]),
            '--yaw', str(base_link2livox_rpy[2]),
            '--frame-id', 'base_link',
            '--child-frame-id', 'livox_frame'
        ]
    )

    # ========================== Sensor Processing Nodes ==========================

    # Ground segmentation node
    bringup_ground_segmentation_node = Node(
        package='linefit_ground_segmentation_ros',
        executable='ground_segmentation_node',
        name='ground_segmentation',
        output='screen',
        parameters=[segmentation_params, {'use_sim_time': use_sim_time}]
    )

    # Pointcloud to laserscan node
    bringup_pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/segmentation/obstacle'),
            ('scan', '/scan')
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': 'livox_frame',
            'transform_tolerance': 0.01,
            'min_height': -1.0,
            'max_height': 0.1,
            'angle_min': -3.14159,   # -M_PI
            'angle_max': 3.14159,    # M_PI
            'angle_increment': 0.0043,  # M_PI/360.0
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }]
    )

    # ========================== FAST_LIO Odometry ==========================

    # FAST_LIO node (publishes lidar_odom -> livox_frame TF)
    bringup_fastlio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[
            fastlio_mid360_params,
            {'use_sim_time': use_sim_time}
        ]
    )

    # FAST_LIO RViz (optional)
    bringup_fastlio_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='lio_rviz',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', fastlio_rviz_cfg_dir],
        condition=IfCondition(use_lio_rviz)
    )

    # ========================== Localization ==========================

    # Map server (lifecycle-managed, provides /map to AMCL)
    # Each launch file creates its own lifecycle_manager_localization,
    # ROS2 allows same-named nodes to coexist — they manage different node lists independently.
    start_map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation2_launch_dir, 'map_server_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': nav2_map_dir,
            'params_file': nav2_params_file_dir,
            'container_name': 'nav2_container'
        }.items()
    )

    # AMCL localization (lifecycle-managed, publishes map -> odom TF)
    start_amcl_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation2_launch_dir, 'localization_amcl_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file_dir
        }.items()
    )

    # ========================== Navigation Stack ==========================

    # Navigation2 stack
    start_navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation2_launch_dir, 'bringup_rm_navigation.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': nav2_map_dir,
            'params_file': nav2_params_file_dir,
            'nav_rviz': use_nav_rviz
        }.items()
    )

    # ========================== Launch Description ==========================
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_lio_rviz_cmd)
    ld.add_action(declare_nav_rviz_cmd)

    # TF transforms (must be first)
    ld.add_action(tf_base_link_to_livox)       # 1. base_link -> livox_frame (static)

    # Sensor processing
    ld.add_action(bringup_ground_segmentation_node)      # 3. Ground segmentation
    ld.add_action(bringup_pointcloud_to_laserscan_node)  # 4. Pointcloud to laserscan

    # Odometry
    ld.add_action(bringup_fastlio_node)        # 5. FAST_LIO (lidar_odom -> livox_frame)
    ld.add_action(bringup_fastlio_rviz)        # 6. FAST_LIO RViz (optional)

    # Localization
    ld.add_action(start_map_server)            # 7. Map server
    ld.add_action(start_amcl_localization)     # 8. AMCL (map -> odom)

    # Navigation
    ld.add_action(start_navigation2)           # 9. Navigation2 stack

    return ld

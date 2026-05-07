import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    # Get the launch directory
    robot_navigation_dir = get_package_share_directory('robot_navigation')
    navigation2_launch_dir = os.path.join(get_package_share_directory('nav2_wrapper'), 'launch')
    small_gicp_launch_dir = os.path.join(
        get_package_share_directory('small_gicp_relocalization'), 'launch')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_lio_rviz = LaunchConfiguration('lio_rviz')
    use_nav_rviz = LaunchConfiguration('nav_rviz')

    # ========================== Parameters Path ==========================

    # FAST_LIO parameters
    fastlio_mid360_params = os.path.join(
        robot_navigation_dir, 'config', 'fastlio_mid360.yaml')
    fastlio_rviz_cfg_dir = os.path.join(
        robot_navigation_dir, 'rviz', 'fastlio.rviz')
    segmentation_params = os.path.join(
        robot_navigation_dir, 'config', 'segmentation_real.yaml')

    # Navigation2 parameters
    nav2_map_dir = os.path.join(robot_navigation_dir, 'map', 'grid_map.yaml')
    nav2_params_file_dir = os.path.join(
        robot_navigation_dir, 'config', 'nav2_params.yaml')
    prior_pcd_file = os.path.join(robot_navigation_dir, 'PCD', 'map.pcd')

    # ========================== Launch Arguments ==========================
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')


    declare_use_lio_rviz_cmd = DeclareLaunchArgument(
        'lio_rviz',
        default_value='False',
        description='Visualize FAST_LIO cloud_map if true')

    declare_nav_rviz_cmd = DeclareLaunchArgument(
        'nav_rviz',
        default_value='True',
        description='Visualize navigation2 if true')

 


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
            'target_frame': 'base_footprint',
            'transform_tolerance': 0.01,
            'min_height': 0.05,
            'max_height': 1.5,
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

    # FAST_LIO node 
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

    odom_to_lidar_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_lidar_odom_tf',
        arguments=[
            '--frame-id', 'odom',
            '--child-frame-id', 'lidar_odom'
        ]
    )

    # ========================== Localization ==========================

    # Map server (lifecycle-managed, provides /map to Nav2 costmaps)
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




    start_small_gicp_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                small_gicp_launch_dir, 'small_gicp_relocalization_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'odom',
            'lidar_frame': 'lidar_odom',
            'robot_base_frame': 'base_footprint',
            'prior_pcd_file': prior_pcd_file,
            'input_cloud_topic': '/cloud_registered',
            'force_2d_transform': 'true'
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

   
    # Sensor processing
    ld.add_action(odom_to_lidar_odom_tf)          # 1. Static odom -> lidar_odom
    ld.add_action(bringup_fastlio_node)           # 2. FAST_LIO (lidar_odom -> base_footprint)
    ld.add_action(bringup_fastlio_rviz)           # Optional FAST_LIO RViz
    ld.add_action(bringup_ground_segmentation_node)      # 3. Ground segmentation
    ld.add_action(bringup_pointcloud_to_laserscan_node)  # 4. Pointcloud to laserscan

    # Localization
    ld.add_action(start_map_server)             # 5. Map server
    ld.add_action(start_small_gicp_localization)  # 6. small_gicp (map -> odom)

    # Navigation
    ld.add_action(start_navigation2)           # 7. Navigation2 stack

    return ld

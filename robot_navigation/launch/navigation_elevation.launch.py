"""
导航 + 高程图集成 Launch 文件

数据流:
  Livox LiDAR → elevation_mapping → /elevation_map (GridMap)
                                          ↓
                        elevation_to_costmap_node
                                          ↓
                              /elevation_costmap (OccupancyGrid)
                                          ↓
                        Nav2 local_costmap (static_layer 订阅)

与 navigation.launch.py 的区别:
  - 新增 elevation_mapping 节点
  - 新增 elevation_to_costmap 转换节点
  - 使用 nav2_params_elevation_sim.yaml 配置文件
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ========================== Package Directories ==========================
    robot_navigation_dir = get_package_share_directory('robot_navigation')
    navigation2_launch_dir = os.path.join(get_package_share_directory('rm_navigation'), 'launch')
    elevation_mapping_pkg = FindPackageShare('elevation_mapping_livox_lidar')

    # ========================== Launch Configurations ==========================
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_lio_rviz = LaunchConfiguration('lio_rviz')
    use_nav_rviz = LaunchConfiguration('nav_rviz')

    # ========================== Parameters Path ==========================
    # Ground segmentation (保留用于 pointcloud_to_laserscan)
    segmentation_params = os.path.join(
        robot_navigation_dir, 'config', 'simulation', 'segmentation_sim.yaml')

    # FAST_LIO parameters
    fastlio_mid360_params = os.path.join(
        robot_navigation_dir, 'config', 'simulation', 'fastlio_mid360_sim.yaml')
    fastlio_rviz_cfg_dir = os.path.join(
        robot_navigation_dir, 'rviz', 'fastlio.rviz')

    # Navigation2 parameters - 使用新的 elevation 配置
    nav2_map_dir = os.path.join(robot_navigation_dir, 'map', 'grid_map.yaml')
    nav2_params_file_dir = os.path.join(
        robot_navigation_dir, 'config', 'simulation', 'nav2_params_elevation_sim.yaml')

    # Elevation mapping parameters
    elevation_robot_config = PathJoinSubstitution([
        elevation_mapping_pkg, 'config', 'elevation_mapping', 'livox_robot.yaml'
    ])
    elevation_map_config = PathJoinSubstitution([
        elevation_mapping_pkg, 'config', 'elevation_mapping', 'livox_map.yaml'
    ])
    elevation_postprocessing_config = PathJoinSubstitution([
        elevation_mapping_pkg, 'config', 'elevation_mapping', 'postprocessor_pipeline.yaml'
    ])

    # base_link -> livox_frame transform
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
    # 注意: 不再发布 base_link → base_footprint 静态变换
    # robot_state_publisher 已从 URDF 发布 base_footprint → base_link,
    # 再反向发布会与之形成 TF 循环冲突, 导致 TF 缓存损坏

    # base_link → livox_frame
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

    # ========================== Sensor Processing ==========================
    # Ground segmentation (用于 pointcloud_to_laserscan → AMCL)
    bringup_ground_segmentation_node = Node(
        package='linefit_ground_segmentation_ros',
        executable='ground_segmentation_node',
        name='ground_segmentation',
        output='screen',
        parameters=[segmentation_params, {'use_sim_time': use_sim_time}]
    )

    # Pointcloud to laserscan (用于 AMCL 定位)
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
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0043,
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }]
    )

    # ========================== FAST_LIO Odometry ==========================
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

    bringup_fastlio_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='lio_rviz',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', fastlio_rviz_cfg_dir],
        condition=IfCondition(use_lio_rviz)
    )

    # ========================== Elevation Mapping ==========================
    # 高程图节点 - 订阅点云，生成高程地图
    elevation_mapping_node = Node(
        package='elevation_mapping',
        executable='elevation_mapping',
        name='elevation_mapping',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            elevation_robot_config,
            elevation_map_config,
            elevation_postprocessing_config,
        ]
    )

    # ========================== AMCL No-Motion Update Trigger ==========================
    # 修复: elevation_mapping 增加 CPU 负载 → FAST-LIO TF 滞后 → AMCL handleInitialPose
    # TF 查询失败 fallback 到单位矩阵 → latest_tf_valid_ 不为 true → 静止时不发布 map→odom
    # 此节点监听 /initialpose，自动调用 /request_nomotion_update 强制粒子滤波更新
    amcl_nomotion_trigger_node = Node(
        package='robot_navigation',
        executable='amcl_nomotion_trigger.py',
        name='amcl_nomotion_trigger',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'delay_sec': 1.0,       # 等待 AMCL 处理完初始位姿
            'trigger_count': 3,     # 连续触发 3 次确保成功
        }]
    )

    # ========================== Elevation to Costmap Converter ==========================
    # 直接从 elevation 层计算坡度，生成 OccupancyGrid
    # 不依赖 traversability 层（elevation_mapping_ros2 的后处理管线未实现）
    elevation_to_costmap_node = Node(
        package='robot_navigation',
        executable='elevation_to_costmap_node',
        name='elevation_to_costmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/elevation_map',     # elevation_mapping 输出
            'output_topic': '/elevation_costmap', # 输出的 OccupancyGrid
            'min_slope_rad': 0.2618,             # 15° 坡度死区（低于此角度 = free）
            'max_slope_rad': 0.5236,             # 30° 最大坡度（高于此角度 = lethal）
            'smooth_radius': 2,                  # 平滑核半径 (5x5 kernel)
            'min_smooth_count': 3,               # 平滑核最少有效邻居数
        }]
    )

    # ========================== Localization ==========================
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

    start_amcl_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation2_launch_dir, 'localization_amcl_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file_dir
        }.items()
    )

    # ========================== Navigation Stack ==========================
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

    # TF transforms
    ld.add_action(tf_base_link_to_livox)

    # Sensor processing
    ld.add_action(bringup_ground_segmentation_node)
    ld.add_action(bringup_pointcloud_to_laserscan_node)

    # Odometry
    ld.add_action(bringup_fastlio_node)
    ld.add_action(bringup_fastlio_rviz)

    # Elevation mapping (延迟启动，等待 TF 树建立)
    ld.add_action(TimerAction(
        period=3.0,
        actions=[elevation_mapping_node]
    ))

    # Elevation to costmap converter (延迟启动，等待 elevation_mapping)
    ld.add_action(TimerAction(
        period=5.0,
        actions=[elevation_to_costmap_node]
    ))

    # Localization
    ld.add_action(start_map_server)
    ld.add_action(start_amcl_localization)

    # AMCL nomotion update trigger (与 AMCL 同步启动)
    ld.add_action(amcl_nomotion_trigger_node)

    # Navigation
    ld.add_action(start_navigation2)

    return ld

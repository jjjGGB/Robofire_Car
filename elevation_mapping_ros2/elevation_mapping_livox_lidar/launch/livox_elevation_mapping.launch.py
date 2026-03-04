#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成启动描述"""

    pkg_share = FindPackageShare('elevation_mapping_livox_lidar')


    # 是否使用仿真时间（Gazebo 仿真必须设为 true）
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时间（Gazebo 仿真必须为 true）'
    )

    # 是否启动 RViz 可视化
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='false',
        description='启动 RViz 可视化界面'
    )

    # 是否启用点云体素滤波（降采样）
    enable_voxel_filter_arg = DeclareLaunchArgument(
        'enable_voxel_filter',
        default_value='true',
        description='启用点云体素滤波降采样'
    )


    # 机器人与传感器配置
    robot_config = PathJoinSubstitution([
        pkg_share, 'config', 'elevation_mapping', 'livox_robot.yaml'
    ])

    # 地图参数配置
    map_config = PathJoinSubstitution([
        pkg_share, 'config', 'elevation_mapping', 'livox_map.yaml'
    ])

    # 后处理管道配置
    postprocessing_config = PathJoinSubstitution([
        pkg_share, 'config', 'elevation_mapping', 'postprocessor_pipeline.yaml'
    ])

    # RViz 配置
    rviz_config = PathJoinSubstitution([
        pkg_share, 'rviz', 'livox_elevation_mapping.rviz'
    ])

    # ==========================================================================
    # 节点定义
    # ==========================================================================

    # --------------------------------------------------------------------------
    # 体素网格滤波节点
    # --------------------------------------------------------------------------
    # 对 Livox 点云进行降采样，减少计算量
    # 注意：使用 /livox/lidar_PointCloud2（标准格式），而非 /livox/lidar（CustomMsg 格式）
    voxel_grid_filter_node = Node(
        package='elevation_mapping_livox_lidar',
        executable='voxel_grid_filter.py',
        name='voxel_grid_filter',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # 体素大小（单位：米）
            # 值越大降采样越多，点云越稀疏
            'leaf_size': 0.05,
            # Z 轴过滤
            'filter_field_name': 'z',
            'filter_limit_min': -1.0,   # 最小高度
            'filter_limit_max': 5.0,    # 最大高度
            'filter_limit_negative': False,
            # 话题配置
            # 重要：必须使用 _PointCloud2 后缀的话题！
            'input_topic': '/livox/lidar_PointCloud2',
            'output_topic': '/livox/lidar_downsampled',
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_voxel_filter'))
    )

    # --------------------------------------------------------------------------
    # 高程图节点
    # --------------------------------------------------------------------------
    # 核心节点，订阅点云并构建高程地图
    elevation_mapping_node = Node(
        package='elevation_mapping',
        executable='elevation_mapping',
        name='elevation_mapping',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            robot_config,           # 机器人与传感器配置
            map_config,             # 地图参数配置
            postprocessing_config,  # 后处理管道配置
        ],
        # 如果启用体素滤波，可以重映射输入话题（当前配置文件直接使用原始话题）
        # remappings=[
        #     ('/livox/lidar_PointCloud2', '/livox/lidar_downsampled'),
        # ],
    )

    # --------------------------------------------------------------------------
    # RViz 可视化节点
    # --------------------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )

    # ==========================================================================
    # 构建启动描述
    # ==========================================================================
    return LaunchDescription([
        # 启动参数
        use_sim_time_arg,
        enable_rviz_arg,
        enable_voxel_filter_arg,

        # 体素滤波节点（立即启动）
        voxel_grid_filter_node,

        # 高程图节点（延迟 2 秒，等待 TF 树建立）
        TimerAction(
            period=2.0,
            actions=[elevation_mapping_node]
        ),

        # RViz 节点（延迟 3 秒）
        TimerAction(
            period=3.0,
            actions=[rviz_node]
        ),
    ])

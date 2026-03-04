#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition


def generate_launch_description():
    # ========== 路径配置 ==========
    # 获取gazebo_ros功能包的路径，用于调用Gazebo的标准launch文件
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # 获取当前功能包（gz_simulation）的路径
    pkg_gz_simulation = get_package_share_directory('gz_simulation')

    # 设置GAZEBO_MODEL_PATH环境变量，使Gazebo能找到自定义模型
    models_path = os.path.join(pkg_gz_simulation, 'models')
    set_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', models_path)

    # 添加world文件选择参数
    world_name = LaunchConfiguration('world_name')
    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='3d_terrain_world.world',
        description='Name of the world file to load (e.g., 3d_terrain_world.world or custom_room.world)'
    )

    # worlds目录的路径 - 使用参数化的world文件名
    world_path = PathJoinSubstitution([
        pkg_gz_simulation,
        'worlds',
        world_name
    ])



    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
        output='screen')

    # ========== 构建Launch描述 ==========
    return LaunchDescription([
        set_model_path,        # 关键：设置模型路径
        declare_world_name,
        start_gazebo_cmd
    ])

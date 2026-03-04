#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ========== 路径配置 ==========
    # 获取当前功能包的launch目录路径
    launch_file_dir = os.path.join(
        get_package_share_directory('gz_simulation'),
        'launch'
    )


    # ========== 子Launch文件包含 ==========
    # 1. 启动Gazebo世界环境
    launch_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'empty_world.launch.py')
        )
    )

    # 2. 生成机器人模型（延迟10秒，等待heightmap完全加载）
    # 负责发布机器人描述并在Gazebo中spawn机器人实体
    launch_robot_spawn = TimerAction(
        period=10.0,  # 延迟10秒
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_file_dir, 'robot_spawn.launch.py')
                )
            )
        ]
    )


    # ========== 构建Launch描述 ==========
    return LaunchDescription([
        launch_empty_world,    # 先启动世界环境
        launch_robot_spawn,    # 延迟后生成机器人

    ])

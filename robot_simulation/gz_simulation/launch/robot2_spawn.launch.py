import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.descriptions
import launch


def generate_launch_description():
    pkg_share = FindPackageShare(package='gz_simulation').find('gz_simulation')
    urdf_file = os.path.join('urdf/simulation_waking_robot.xacro')
    set_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', pkg_share)

    # ---- robot state publisher ----
    # Unique node name + topic to avoid collision with robot1
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot2_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': launch_ros.descriptions.ParameterValue(
                launch.substitutions.Command([
                    'xacro ', os.path.join(pkg_share, urdf_file)]),
                value_type=str)
        }],
        remappings=[('robot_description', 'robot2/robot_description')],
    )

    # ---- spawn entity in Gazebo ----
    # gazebo_ros_four_wheel_steering and joint_state_publisher plugins
    # are loaded automatically via the URDF/xacro <gazebo> blocks.
    # No ros2_control spawners needed.
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot2',
        arguments=[
            '-entity', LaunchConfiguration('robot_name'),
            '-topic', 'robot2/robot_description',
            '-x', LaunchConfiguration('start_x'),
            '-y', LaunchConfiguration('start_y'),
            '-z', LaunchConfiguration('start_z'),
            '-Y', LaunchConfiguration('start_yaw'),
            '-timeout', '1000'
        ],
        output='screen',
    )


    return LaunchDescription([
        set_model_path,
        DeclareLaunchArgument('start_x', default_value='0.17',
                              description='X coordinate of starting position'),
        DeclareLaunchArgument('start_y', default_value='0.0',
                              description='Y coordinate of starting position'),
        DeclareLaunchArgument('start_z', default_value='0.5',  
                              description='Z coordinate of starting position'),
        DeclareLaunchArgument('start_yaw', default_value='0.0',
                              description='Yaw angle of starting orientation'),
        DeclareLaunchArgument('robot_name', default_value='myrobot2',
                              description='Name and prefix for this robot'),

        robot_state_publisher_node,
        spawn_entity_node,

    ])

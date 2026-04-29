import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.descriptions
import launch


def generate_launch_description():
    pkg_share = FindPackageShare(package='vehicle_chassis').find('vehicle_chassis')
    urdf_file = os.path.join('urdf/vehicle_chassis.urdf.xacro')
    set_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', pkg_share)

    # ---- robot state publisher ----
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': launch_ros.descriptions.ParameterValue(
                launch.substitutions.Command([
                    'xacro ', os.path.join(pkg_share, urdf_file),
                    ' scale_factor:=', LaunchConfiguration('scale_factor')]),
                value_type=str)
        }]
    )

    # ---- spawn entity in Gazebo ----
    # gazebo_ros_four_wheel_steering and joint_state_publisher plugins
    # are loaded automatically via the URDF/xacro <gazebo> blocks.
    # No ros2_control spawners needed.
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', LaunchConfiguration('robot_name'),
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('start_x'),
            '-y', LaunchConfiguration('start_y'),
            '-z', LaunchConfiguration('start_z'),
            '-Y', LaunchConfiguration('start_yaw'),
            '-timeout', '1000'
        ],
        output='screen',
    )

    # ---- cmd_vel -> cmd_4ws bridge (for Nav2 integration) ----
    cmd_vel_bridge_node = Node(
        package='vehicle_chassis',
        executable='cmd_vel_bridge',
        name='cmd_vel_to_4ws',
        parameters=[{
            'use_sim_time': True,
            'wheelbase': PythonExpression([LaunchConfiguration('scale_factor'), ' * 2.071']),
            'max_steering_angle': 0.75,
        }],
        output='screen',
    )

    return LaunchDescription([
        set_model_path,
        DeclareLaunchArgument('start_x', default_value='0.0',
                              description='X coordinate of starting position'),
        DeclareLaunchArgument('start_y', default_value='0.0',
                              description='Y coordinate of starting position'),
        DeclareLaunchArgument('start_z', default_value='0.5',
                              description='Z coordinate of starting position'),
        DeclareLaunchArgument('start_yaw', default_value='0.0',
                              description='Yaw angle of starting orientation'),
        DeclareLaunchArgument('robot_name', default_value='myrobot',
                              description='Name and prefix for this robot'),
        DeclareLaunchArgument('scale_factor', default_value='1.0',
                              description='Uniform vehicle scale factor'),

        robot_state_publisher_node,
        spawn_entity_node,
        cmd_vel_bridge_node,
    ])


import os

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    # Getting directories and launch-files
    bringup_dir = get_package_share_directory('linefit_ground_segmentation_ros')
    params_file = os.path.join(bringup_dir, 'launch', 'segmentation_params.yaml')
    visualize = LaunchConfiguration('visualize')

    # Nodes launching commands
    node_start_cmd = Node(
            package='linefit_ground_segmentation_ros',
            executable='ground_segmentation_node',
            output='screen',
            parameters=[params_file, {'visualize': visualize}])


    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(DeclareLaunchArgument(
        'visualize',
        default_value='false',
        description='Enable the built-in PCL/VTK debug viewer. Requires a working local OpenGL/GLX display.'
    ))
    ld.add_action(node_start_cmd)


    return ld

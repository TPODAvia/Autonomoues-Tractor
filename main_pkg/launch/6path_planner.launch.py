# full_coverage_path_planner is not support for ros2 humble yet

import os
 
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    
    slam_path = PythonLaunchDescriptionSource([get_package_share_directory('my_package'), '/launch/2slam.launch.py']) 
    
    return LaunchDescription([
        # Node(
        #     package='full_coverage_path_planner',
        #     executable='full_coverage_path_planner',
        #     name='full_coverage_path_planner',
        #     output='screen',
        # ),
        # Get the path to the launch file
        IncludeLaunchDescription(slam_path),

    ])
# ros2 launch mpu6050driver mpu6050driver_launch.py
# ros2 run gpsx gps_node

import os
 
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
 
 
 
    return LaunchDescription([
        Node(
            package='mpu6050driver',
            executable='mpu6050driver_launch.py',
            name='mpu6050driver',
            output='screen',
        ),
        Node(
            package='gpsx',
            executable='gps_node',
            name='gpsx',
            output='screen',
        ),
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'ekf.yaml')],
        )
    ])
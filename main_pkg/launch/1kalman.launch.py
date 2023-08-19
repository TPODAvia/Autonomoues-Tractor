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
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
 
    ld = LaunchDescription()
    share_dir = get_package_share_directory('mpu6050driver')
    parameter_file = LaunchConfiguration('params_file')

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'mpu6050.yaml'),
                                           description='Path to the ROS2 parameters file to use.')

    mpu6050driver_node = Node(
        package='mpu6050driver',
        executable='mpu6050driver',
        name='mpu6050driver_node',
        output="screen",
        emulate_tty=True,
        parameters=[parameter_file]
    )

    ld.add_action(params_declare)
    ld.add_action(mpu6050driver_node)
 
    return LaunchDescription([
        ld,

        Node(
            package='main_pkg',
            executable='fake_gps.py',
            name='fake_gps'
        ),

        # Node(
        #     package='gpsx',
        #     executable='gps_node',
        #     name='gpsx',
        #     output='screen',
        # ),


        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("main_pkg"), 'launch', 'ekf.yaml')],
        )
    ])
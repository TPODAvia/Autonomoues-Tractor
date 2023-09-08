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
    share_dir = get_package_share_directory('mpu9250driver')
    parameter_file = LaunchConfiguration('params_file')

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'mpu9250.yaml'),
                                           description='Path to the ROS2 parameters file to use.')

    mpu9250driver_node = Node(
        package='mpu9250driver',
        executable='mpu9250driver',
        name='mpu9250driver_node',
        # output="screen",
        emulate_tty=True,
        parameters=[parameter_file]
    )

    ld.add_action(params_declare)
    ld.add_action(mpu9250driver_node)

    return LaunchDescription([
        ld,

        Node(
            package='main_pkg',
            executable='real_gps.py',
            name='real_gps',
            output='screen',
            parameters=["/dev/ttyUSB0"],
        ),

        #Node(
        #    package='main_pkg',
        #    executable='fake_gps.py',
        #    name='fake_gps',
        #    output='screen',
        #),

        Node(
            package='main_pkg',
            executable='web.py',
            name='web_node',
            output='screen',
        ),

        Node(
            package='main_pkg',
            executable='odom.py',
            name='odom_node',
            output='screen',
        ),

        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("main_pkg"), 'launch', 'ekf.yaml')],
            remappings=[('example/imu', 'imu'),
                        ('example/odom', 'odometry/filtered')]      
           ),

        launch_ros.actions.Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("main_pkg"), 'launch', 'navsat_transform.yaml')],
           ),
    ])
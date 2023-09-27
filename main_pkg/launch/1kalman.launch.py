# ros2 launch mpu6050driver mpu6050driver_launch.py
# ros2 run gpsx gps_node

import os
from launch.actions import TimerAction
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
# import launch_ros.actions
# import yaml
# from launch.substitutions import EnvironmentVariable
# import pathlib
# import launch.actions
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration

def generate_launch_description():
     
    config_dir = os.path.join(get_package_share_directory('main_pkg'), 'launch')

    return LaunchDescription([
        Node(
            package='main_pkg',
            executable='mpu9250_node.py',
            name='mpu9250',     
        ),

        Node(
            package='main_pkg',
            executable='real_gps.py',
            name='real_gps',
            output='screen',
            parameters=[{'serial_port': '/dev/ttyACM0'}],
        ),

        # Node(
        #    package='main_pkg',
        #    executable='fake_gps.py',
        #    name='fake_gps',
        #    output='screen',
        # ),

        Node(
            package='main_pkg',
            executable='web.py',
            name='web_node',
            output='screen',
        ),

        # Node(
        #     package='main_pkg',
        #     executable='fake_odom.py',
        #     name='fake_odom_node',
        #     output='screen',
        # ),

        TimerAction(period=24.0, actions=[
            Node(
                package='robot_localization',
                executable='navsat_transform_node',
                name='navsat_transform_node',
                output='screen',
                parameters=[os.path.join(config_dir, 'navsat_transform.yaml')],
            ),
        ]),

        TimerAction(period=22.0, actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[os.path.join(config_dir, 'ekf.yaml')],
            ),
        ]),

        TimerAction(period=22.0, actions=[
            Node(
                package='imu_filter_madgwick',
                executable='imu_filter_madgwick_node',
                name='imu_filter',
                output='screen',
                parameters=[os.path.join(config_dir, 'imu_filter.yaml')],
                remappings=[('imu/data', 'imu')] 
            ),
        ]),
        
        # Node(
        #     package='rviz2',
        #     namespace='',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d' + os.path.join(get_package_share_directory('main_pkg'), 'config', 'config_file.rviz')]
        # )
    ])
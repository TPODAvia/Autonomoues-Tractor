import os
from launch.actions import TimerAction
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
     
    config_dir = os.path.join(get_package_share_directory('main_pkg'), 'launch')

    use_real_gps = LaunchConfiguration('use_real_gps')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_real_gps',
        default_value='False',
        description='To run real gps hardware turn this to true')
    
    return LaunchDescription([
        declare_use_rviz_cmd,

        Node(
            package='main_pkg',
            executable='mpu9250_node.py',
            name='mpu9250',     
        ),

        Node(
            condition=IfCondition(use_real_gps),
            package='main_pkg',
            executable='real_gps.py',
            name='real_gps',
            output='screen',
            parameters=[{'serial_port': '/dev/ttyACM0'}],
        ),

        Node(
            condition=IfCondition(PythonExpression(['not ', use_real_gps])),
            package='main_pkg',
            executable='fake_gps.py',
            name='fake_gps',
            output='screen',
        ),

        # Node(
        #     package='main_pkg',
        #     executable='web.py',
        #     name='web_node',
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

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.5', '0.5', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
            parameters=[{'use_sim_time': True}]
        ),

    ])
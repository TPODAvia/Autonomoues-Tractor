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


    robot_localization_dir = get_package_share_directory('main_pkg')
    parameters_file_dir = os.path.join(robot_localization_dir, 'launch')
    parameters_file_path = os.path.join(parameters_file_dir, 'dual_ekf_navsat_example.yaml')
    os.environ['FILE_PATH'] = str(parameters_file_dir)
    serial_port = "/dev/ttyACM0"

    return LaunchDescription([
        ld,

        Node(
            package='main_pkg',
            executable='real_gps.py',
            name='real_gps',
            output='screen',
            parameters=[serial_port],
        ),

        Node(
            package='main_pkg',
            executable='web.py',
            name='web_node',
            output='screen',
        ),

          
        launch_ros.actions.Node(
                package='robot_localization', 
                executable='navsat_transform_node', 
                name='navsat_transform',
                output='screen',
                parameters=[parameters_file_path],
                remappings=[('imu/data', 'imu/data'),
                            ('gps/fix', 'gps/fix'), 
                            ('gps/filtered', 'gps/filtered'),
                            ('odometry/gps', 'odometry/gps'),
                            ('odometry/filtered', 'odometry/global')]           

            )
    ])
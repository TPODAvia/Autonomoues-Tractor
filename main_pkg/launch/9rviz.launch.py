import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
     
    config_dir = os.path.join(get_package_share_directory('main_pkg'), 'launch')

    return LaunchDescription([
        
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('main_pkg'), 'config', 'config_file.rviz')]
        )
    ])
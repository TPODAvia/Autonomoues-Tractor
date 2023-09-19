import os
 
from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
 
    return LaunchDescription([
        Node(
            package='main_pkg',
            executable='control_node.py',
            name='control_node'
        )
    ])
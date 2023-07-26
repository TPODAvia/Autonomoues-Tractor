# ros2 run teleop_twist_keyboard teleop_twist_keyboard
# ros2 launch my_cpp_py_pkg camera.launch.py
# ros2 launch my_cpp_py_pkg web_control.launch.py
# http://10.100.190.77:8080/
import os
 
from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
 
    return LaunchDescription([
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen',
        ),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            output='screen',
            parameters=[{
                'image_size': [640,480],
                'camera_frame_id': 'camera_link_optical'
                }]
        ),
        Node(
            package='my_cpp_py_pkg',
            executable='cpp_executable',
            name='my_node'
        )
    ])
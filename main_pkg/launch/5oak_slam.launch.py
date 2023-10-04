import os
# Do rectification
# Do image syncronisation


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch.actions import TimerAction

def launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration('name').perform(context)
    depthai_prefix = get_package_share_directory("depthai_ros_driver")

    params_file= LaunchConfiguration("params_file")
    parameters = [
        {
            "frame_id": name,
            "subscribe_rgb": True,
            "subscribe_depth": True,
            "subscribe_odom_info": True,
            "approx_sync": True,
            "Rtabmap/DetectionRate": "3.5",
        }
    ]

    remappings = [
        ("rgb/image", name+"/rgb/image_raw"),
        ("rgb/camera_info", name+"/rgb/camera_info"),
        ("depth/image", name+"/stereo/image_raw"),
    ]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, 'launch', 'camera.launch.py')),
            launch_arguments={"name": name,
                              "params_file": params_file}.items()),

        # LoadComposableNodes(
        #     condition=IfCondition(LaunchConfiguration("rectify_rgb")),
        #     target_container=name+"_container",
        #     composable_node_descriptions=[
        #         ComposableNode(
        #             package="image_proc",
        #             plugin="image_proc::RectifyNode",
        #             name="rectify_color_node",
        #             remappings=[('image', name+'/rgb/image_raw'),
        #                         ('camera_info', name+'/rgb/camera_info'),
        #                         ('image_rect', name+'/rgb/image_rect'),
        #                         ('image_rect/compressed', name+'/rgb/image_rect/compressed'),
        #                         ('image_rect/compressedDepth', name+'/rgb/image_rect/compressedDepth'),
        #                         ('image_rect/theora', name+'/rgb/image_rect/theora')]
        #         )
        #     ]),
        
        LoadComposableNodes(
            target_container=name+"_container",
            composable_node_descriptions=[
                ComposableNode(
                    package='rtabmap_odom',
                    plugin='rtabmap_odom::RGBDOdometry',
                    name='rgbd_odometry',
                    parameters=parameters,
                    remappings=remappings,
                ),
            ],
        ),

        # LoadComposableNodes(
        #     target_container=name+"_container",
        #     composable_node_descriptions=[
        #         ComposableNode(
        #             package='rtabmap_slam',
        #             plugin='rtabmap_slam::CoreWrapper',
        #             name='rtabmap',
        #             parameters=parameters,
        #             remappings=remappings,
        #         ),
        #     ],
        # ),

        # Node(
        #     package="rtabmap_viz",
        #     executable="rtabmap_viz",
        #     output="screen",
        #     parameters=parameters,
        #     remappings=remappings,
        # ),
    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    config_dir = os.path.join(get_package_share_directory('main_pkg'), 'launch')
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(depthai_prefix, 'config', 'rgbd.yaml')),
        DeclareLaunchArgument("rectify_rgb", default_value="True"),
    ]

    # imu_node = Node(
    #         package='main_pkg',
    #         executable='mpu9250_node.py',
    #         name='mpu9250', 
    #     )

    # madwick_node = TimerAction(period=22.0, actions=[
    #     Node(
    #         package='imu_filter_madgwick',
    #         executable='imu_filter_madgwick_node',
    #         name='imu_filter',
    #         output='screen',
    #         parameters=[os.path.join(config_dir, 'imu_filter.yaml')],
    #         remappings=[('imu/data', 'imu')] 
    #     ),
    
    # ])

    parameters={
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'subscribe_rgbd':True,
          'subscribe_scan':True,
          'use_action_for_goal':True,
          'qos_scan':qos,
          'qos_image':qos,
          'qos_imu':qos,
          # RTAB-Map's parameters should be strings:
          'Reg/Strategy':'1',
          'Reg/Force3DoF':'true',
          'RGBD/NeighborLinkRefining':'True',
          'Grid/RangeMin':'0.2', # ignore laser scan points on the robot itself
          'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    
    remappings=[
          ('rgb/image', '/camera/image_raw'),
          ('rgb/camera_info', '/camera/camera_info'),
          ('depth/image', '/camera/depth/image_raw')]

    Node(
        package='rtabmap_sync', executable='rgbd_sync', output='screen',
        parameters=[{'approx_sync':False, 'use_sim_time':use_sim_time, 'qos':qos}],
        remappings=remappings),
    
    # static_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0', '1', '0', '0', '0', 'base_link', 'oak']
    # )

    return LaunchDescription(
        # declared_arguments + [OpaqueFunction(function=launch_setup), imu_node, madwick_node, static_tf_node]
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
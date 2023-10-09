import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch.actions import TimerAction

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_rviz = LaunchConfiguration('rviz')
    use_driver = LaunchConfiguration('control_driver')
    use_kalman = LaunchConfiguration('kalman_filter')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)
    
    gps_wpf_dir = get_package_share_directory("main_pkg")
    params_dir = os.path.join(gps_wpf_dir, "launch")
    nav2_params = os.path.join(params_dir, "nav2_params.yaml")
    configured_params = RewrittenYaml(source_file=nav2_params, root_key="", param_rewrites="", convert_types=True )
    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_use_rviz_cmd =          DeclareLaunchArgument('rviz',           default_value='False',  description='Whether to start RVIZ')
    declare_namespace_cmd =         DeclareLaunchArgument('namespace',      default_value='',       description='Top-level namespace')
    declare_use_namespace_cmd =     DeclareLaunchArgument('use_namespace',  default_value='False',  description='Whether to apply a namespace to the navigation stack')
    declare_slam_cmd =              DeclareLaunchArgument('slam',           default_value='False',  description='Whether run a SLAM')
    declare_map_yaml_cmd =          DeclareLaunchArgument('map',            default_value=os.path.join(bringup_dir, 'maps', 'turtlebot3_world.yaml'), description='Full path to map file to load')
    declare_use_sim_time_cmd =      DeclareLaunchArgument('use_sim_time',   default_value='False',  description='Use simulation (Gazebo) clock if true')
    declare_params_file_cmd =       DeclareLaunchArgument('params_file',    default_value=os.path.join(get_package_share_directory('main_pkg'), 'launch', 'nav2_params.yaml'), description='Full path to the ROS2 parameters file to use for all launched nodes')
    declare_autostart_cmd =         DeclareLaunchArgument('autostart',      default_value='True',   description='Automatically startup the nav2 stack')
    declare_use_composition_cmd =   DeclareLaunchArgument('use_composition', default_value='True',  description='Whether to use composed bringup')
    declare_use_respawn_cmd =       DeclareLaunchArgument('use_respawn',    default_value='False',  description='Whether to respawn if a node crashes. Applied when composition is disabled.')
    declare_log_level_cmd =         DeclareLaunchArgument('log_level',      default_value='info',   description='log level')
    declare_use_control_cmd =       DeclareLaunchArgument('control_driver', default_value='False',   description='Control node for the robot')
    declare_use_kalman_cmd =        DeclareLaunchArgument('kalman_filter',  default_value='False',   description='Trigger the EKF node')

    kalman_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gps_wpf_dir, "launch", '1kalman.launch.py')),
        condition=IfCondition(use_kalman)
    )

    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(os.path.join(get_package_share_directory('main_pkg'), 'launch'), '5oak_slam.launch.py')
        ),
        condition=IfCondition(use_slam)
    )
    
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", 'rviz_launch.py')),
        condition=IfCondition(use_rviz)
    )

    control_node = Node(
        package='main_pkg',
        executable='control_node.py',
        name='control_node',
        condition=IfCondition(use_driver)
    )

    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),
        
        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization_launch.py')),
            condition=IfCondition(PythonExpression(['not ', use_slam])),
            launch_arguments={'namespace': namespace,
                              'map': map_yaml_file,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "use_sim_time": "False",
                "params_file": configured_params,
                "autostart": "True",
            }.items(),
        ),
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_use_rviz_cmd)
    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_control_cmd)
    ld.add_action(declare_use_kalman_cmd)

    # Add the actions to launch all of the navigation nodes
    # ld.add_action(TimerAction(period=22.0, actions=[bringup_cmd_group]))

    # Robot localization launch
    ld.add_action(kalman_cmd)

    # Use SLAM
    ld.add_action(TimerAction(period=25.0, actions=[slam_cmd]))

    # Use control driver
    ld.add_action(control_node)

    # Visualization
    ld.add_action(TimerAction(period=30.0, actions=[rviz_cmd]))

    return ld
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    jetbot_bringup_pkg_share = FindPackageShare('jetbot_bringup')

    # Declare the launch argument for robot_id
    declare_robot_id_cmd = DeclareLaunchArgument(
        'robot_namespace',
        default_value='robot_2',
        description='ID of the robot, which is used as namespace.'
    )

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            jetbot_bringup_pkg_share,
            'maps',
            'map_labirint_tbank.yaml',
        ]),
        description='Full path to map YAML (same format as nav2 map_server; image path inside YAML is relative to that file).',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            jetbot_bringup_pkg_share,
            'config',
            'nav2_default_localization_ignore_dyn_obst_test.yaml',
        ]),
        description='Nav2 + map_server + amcl parameter YAML for localization.launch.py',
    )
    declare_odometry_source_cmd = DeclareLaunchArgument(
        'odometry_source',
        default_value='lidar',
        description="Odometry source: 'lidar' (KISS-ICP) or 'encoders'",
    )

    robot_namespace = LaunchConfiguration('robot_namespace')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    odometry_source = LaunchConfiguration('odometry_source')
    
    # Include activate_all_drivers.launch.py
    # Assuming it is in the same package and accepts a 'namespace' argument.
    activate_all_drivers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                jetbot_bringup_pkg_share,
                'launch',
                'activate_all_drivers.launch.py'
            ])
        ),
        launch_arguments={
            'robot_namespace': robot_namespace,
            'odometry_source': odometry_source,
        }.items()
    )

    # Include navig.launch.py - will start 10 seconds after activate_all_drivers
    navig_launch = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        jetbot_bringup_pkg_share,
                        'launch',
                        'localization.launch.py'
                    ])
                ),
                launch_arguments={
                    'robot_namespace': robot_namespace,
                    'map': map_yaml,
                    'params_file': params_file,
                }.items()
            )
        ]
    )

    # Robot navigation bridge - starts 20 seconds after activate_all_drivers (after Nav2)
    robot_nav_bridge_launch = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='jetbot_bringup',
                executable='robot_nav_bridge',
                name='robot_nav_bridge',
                namespace=robot_namespace,
                parameters=[{
                    'robot_namespace': robot_namespace
                }],
                output='screen'
            )
        ]
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_robot_id_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_odometry_source_cmd)
    ld.add_action(activate_all_drivers_launch)
    ld.add_action(navig_launch)
    ld.add_action(robot_nav_bridge_launch)

    return ld

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Declare robot ID argument
    declared_arguments = []
    declared_arguments.append(
       DeclareLaunchArgument(
        'robot_namespace',
        default_value='robot_3',
        description='ID of the robot, which is used as namespace.'
    )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'odometry_source',
            default_value='lidar',
            description="Odometry source: 'lidar' (KISS-ICP) or 'encoders'"
        )
    )
    
    # Initialize Arguments
    robot_namespace = LaunchConfiguration('robot_namespace')
    odometry_source = LaunchConfiguration('odometry_source')
    use_lidar_odom = IfCondition(PythonExpression(["'", odometry_source, "' == 'lidar'"]))
    use_encoder_odom = IfCondition(PythonExpression(["'", odometry_source, "' == 'encoders'"]))

    # Publish a dummy message to create the topic for twist_stamped_to_twist
    topic_name = [robot_namespace, '/cmd_vel_robot_steering_stamped']
    message_type = 'geometry_msgs/msg/TwistStamped'
    message_content = '"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"'
    
    create_topic_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', topic_name, message_type, message_content],
        shell=True
    )

# Needed to activate robot steering app before activation
    twist_stamped_to_twist = Node(
        package='topic_tools',
        executable='relay_field',
        name='twist_stamped_to_twist',
        # namespace=robot_namespace,
        arguments=[
            [robot_namespace, '/cmd_vel_robot_steering_stamped'],
            [robot_namespace, '/cmd_vel_robot_steering'],
            'geometry_msgs/msg/Twist',
            '{linear: m.twist.linear, angular: m.twist.angular}'
        ]
    )

    # Event handler to launch twist_stamped_to_twist after create_topic_cmd finishes
    launch_relay_after_topic_creation = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=create_topic_cmd,
            on_exit=[twist_stamped_to_twist],
        )
    )

    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
        namespace=robot_namespace,
        parameters=[{
            'dev': "/dev/input/js0"
        }]
    )

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        namespace=robot_namespace,
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("jetbot_bringup"), "config", "joy.yaml"
            ]),
            {'publish_stamped_twist': False}
        ],
        remappings=[('cmd_vel', 'cmd_vel_joy')]
    )

    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            namespace=robot_namespace,
            parameters=[PathJoinSubstitution([
                FindPackageShare("jetbot_bringup"), "config", "twist_mux.yaml"
            ])],
            remappings=[('cmd_vel_out', 'diffbot_base_controller/cmd_vel_unstamped')],
            # arguments=['--ros-args', '--log-level', 'debug']
        )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("jetbot_bringup"), "launch", "rplidar.launch.py"
            ])
        ),
        launch_arguments={
            'robot_namespace': robot_namespace,
        }.items()
    )

    diffbot_launch_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("diffdrive_jetbot"), "launch", "diffbot.launch.py"
            ])
        ),
        launch_arguments={
            'robot_namespace': robot_namespace,
            'controllers_file': PathJoinSubstitution([
                FindPackageShare("diffdrive_jetbot"), "config", "diffbot_controllers_lidar_odom.yaml"
            ]),
        }.items(),
        condition=use_lidar_odom
    )

    diffbot_launch_encoders = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("diffdrive_jetbot"), "launch", "diffbot.launch.py"
            ])
        ),
        launch_arguments={
            'robot_namespace': robot_namespace,
            'controllers_file': PathJoinSubstitution([
                FindPackageShare("diffdrive_jetbot"), "config", "diffbot_controllers.yaml"
            ]),
        }.items()
        ,
        condition=use_encoder_odom
    )

    scan_to_cloud = Node(
        package='jetbot_bringup',
        executable='scan_to_cloud',
        name='scan_to_cloud',
        namespace=robot_namespace,
        parameters=[
            {'scan_topic': 'scan'},
            {'cloud_topic': 'scan/points'},
            {'target_frame': [robot_namespace, TextSubstitution(text='/lidar_link')]},
        ],
        condition=use_lidar_odom,
        output='screen',
    )

    kiss_icp = Node(
        package='kiss_icp',
        executable='kiss_icp_node',
        name='kiss_icp_node',
        namespace=robot_namespace,
        remappings=[
            ('pointcloud_topic', 'scan/points'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        parameters=[
            {
                'base_frame': [robot_namespace, TextSubstitution(text='/base_footprint')],
                'lidar_odom_frame': [robot_namespace, TextSubstitution(text='/odom')],
                'publish_odom_tf': True,
                'invert_odom_tf': False,
                'publish_debug_clouds': False,
            }
        ],
        condition=use_lidar_odom,
        output='screen',
    )

    kiss_odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='kiss_odom_relay',
        namespace=robot_namespace,
        arguments=['kiss/odometry', 'odom'],
        condition=use_lidar_odom,
        output='screen',
    )

    encoder_odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='encoder_odom_relay',
        namespace=robot_namespace,
        arguments=['diffbot_base_controller/odom', 'odom'],
        condition=use_encoder_odom,
        output='screen',
    )

    return LaunchDescription(declared_arguments + [
        # create_topic_cmd,
        # launch_relay_after_topic_creation,
        joy_node,
        teleop_twist_joy_node,
        twist_mux,
        lidar_launch,
        scan_to_cloud,
        kiss_icp,
        kiss_odom_relay,
        encoder_odom_relay,
        diffbot_launch_lidar,
        diffbot_launch_encoders,
    ])

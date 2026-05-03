from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
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
    
    # Initialize Arguments
    robot_namespace = LaunchConfiguration('robot_namespace')

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

    diffbot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("diffdrive_jetbot"), "launch", "diffbot.launch.py"
            ])
        ),
        launch_arguments={
            'robot_namespace': robot_namespace,
        }.items()
    )

    return LaunchDescription(declared_arguments + [
        # create_topic_cmd,
        # launch_relay_after_topic_creation,
        joy_node,
        teleop_twist_joy_node,
        twist_mux,
        lidar_launch,
        diffbot_launch,
    ])

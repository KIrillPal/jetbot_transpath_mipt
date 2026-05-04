# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import socket
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml

def get_local_ip():
    """Get the local IP address"""
    try:
        # Connect to a remote address to determine local IP
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_namespace",
            default_value="robot_2",
            description="Unique robot ID (1-5)"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value=get_local_ip(),
            description="IP address of the robot for HTTP mesh server",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mesh_port",
            default_value="8000",
            description="Port for HTTP mesh server",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("diffdrive_jetbot"),
                    "config",
                    "diffbot_controllers.yaml",
                ]
            ),
            description="Path to ros2_control controllers yaml",
        )
    )

    # Initialize Arguments
    robot_namespace = LaunchConfiguration("robot_namespace")
    robot_ip = LaunchConfiguration("robot_ip")
    mesh_port = LaunchConfiguration("mesh_port")
    controllers_file = LaunchConfiguration("controllers_file")
    cur_controller_manager = [TextSubstitution(text="controller_manager")]
    cur_remappings=[("/tf", "tf"),("/tf_static", "tf_static")]

    # Get URDF via xacro with robot_id prefix
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("diffdrive_jetbot"), "urdf", "diffbot.urdf.xacro"]
            ),
            " ",
            "prefix:=",robot_namespace,
            "/ ",
            "robot_ip:=",
            robot_ip,
            " ",
            "mesh_port:=",
            mesh_port,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = controllers_file

    # Start HTTP server for mesh files
    mesh_server = ExecuteProcess(
        cmd=[
            "python3", "-m", "http.server", mesh_port,
            "--directory", PathJoinSubstitution([FindPackageShare("diffdrive_jetbot"), "description"])
        ],
        name="mesh_http_server",
        output="log"
    )

    robot_controllers = ReplaceString(
        source_file=robot_controllers,
        replacements={'<robot_namespace>': (robot_namespace, '/')},
    )

    robot_controllers = ParameterFile(
        RewrittenYaml(
            source_file=robot_controllers,
            root_key=robot_namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=robot_namespace,
        parameters=[robot_description, robot_controllers],
        remappings=cur_remappings,
        output="both",
    )
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=robot_namespace,
        output="both",
        parameters=[robot_description],
        remappings=cur_remappings,
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=robot_namespace,
        remappings=cur_remappings,
arguments=["joint_state_broadcaster", "--controller-manager", cur_controller_manager],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=robot_namespace,
        remappings=cur_remappings,
        arguments=["diffbot_base_controller", "--controller-manager", cur_controller_manager],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        # mesh_server,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)

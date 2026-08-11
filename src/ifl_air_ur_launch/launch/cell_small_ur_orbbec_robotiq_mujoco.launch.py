from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def resolve_runtime_output(quiet_requested, normal_output):
    """Select log-only output for an explicitly quiet runtime."""
    quiet = str(quiet_requested).strip().casefold()
    if quiet in {"1", "true", "yes", "on"}:
        return {"both": "log"}
    return normal_output


def launch_runtime_nodes(context, rviz_config_file, robot_description):
    """Create gripper/robot-state nodes after quiet mode is resolved."""
    quiet_requested = context.perform_substitution(
        LaunchConfiguration("quiet_runtime_output")
    )
    return [
        Node(
            package="robotiq_2f_urcap_adapter",
            executable="robotiq_2f_adapter_node.py",
            name="robotiq_2f_urcap_adapter",
            output=resolve_runtime_output(quiet_requested, "screen"),
            parameters=[{
                "robot_ip": "127.0.0.1",
                "robot_port": 63352,
                "auto_calibrate": False,
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output=resolve_runtime_output(quiet_requested, "log"),
            arguments=["-d", rviz_config_file],
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output=resolve_runtime_output(quiet_requested, "both"),
            parameters=[robot_description],
        ),
    ]



def generate_launch_description():
    # Declare arguments
    args = [
        DeclareLaunchArgument('camera_name', default_value='camera'), # Dummy argument
        DeclareLaunchArgument('launch_rviz', default_value='true', description='Launch RViz'),
        DeclareLaunchArgument(
            'quiet_runtime_output',
            default_value='false',
            description='Write persistent runtime output to logs only',
        ),
    ]
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('ifl_air_description'), "rviz", "view_robot.rviz"]
    )

    description_file = 'cell_small_ur_orbbec_robotiq2f85.urdf.xacro'

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare('ifl_air_description'), "urdf", description_file]),
            " ",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    runtime_nodes = OpaqueFunction(
        function=launch_runtime_nodes,
        args=[rviz_config_file, robot_description],
    )
    return LaunchDescription(args + [runtime_nodes])

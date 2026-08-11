# This launch file is created following the tutorial at 
# https://moveit.picknik.ai/main/doc/how_to_guides/moveit_launch_files/moveit_launch_files_tutorial.html

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory


def resolve_runtime_output(quiet_requested, normal_output):
    """Select log-only output for an explicitly quiet runtime."""
    quiet = str(quiet_requested).strip().casefold()
    if quiet in {"1", "true", "yes", "on"}:
        return {"both": "log"}
    return normal_output


def launch_runtime_nodes(
    context,
    moveit_config,
    move_group_parameters,
):
    """Create MoveIt/RViz nodes after quiet mode has been resolved."""
    quiet_requested = context.perform_substitution(
        LaunchConfiguration("quiet_runtime_output")
    )
    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output=resolve_runtime_output(quiet_requested, "screen"),
            parameters=move_group_parameters,
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_moveit",
            output=resolve_runtime_output(quiet_requested, "log"),
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        ),
    ]


def generate_launch_description():

    # define xacro mapping for the robot description file

    launch_arguments ={
        "ur_type": "ur10e",

    }  

    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            "air_ur_robotiq", package_name="ifl_air_cell_small_ur_orbbec_robotiq_moveit_config"
        )
        .robot_description(mappings=launch_arguments) # optional: can be omitted to use default arguments
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "chomp"]
        ) # optional: can be omitted to use the default pipeline ("stomp" is currently not supported in humble)
        .sensors_3d(file_path=None)
        .to_moveit_configs()
    )

    publish_robot_description_semantic = {"publish_robot_description_semantic": True}
    publish_robot_description = {"publish_robot_description": True}
    publish_robot_description_kinematics = {"publish_robot_description_kinematics": True}

    move_group_parameters = [
        moveit_config.to_dict(),
        publish_robot_description,
        publish_robot_description_kinematics,
        publish_robot_description_semantic,
    ]

    # RViz for visualization
    # Get the path to the RViz config file
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config", 
        default_value=str(moveit_config.package_path / "config/moveit.rviz"), 
        description="RViz config file"
    )

    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Launch RViz2 when true",
    )
    
    runtime_nodes = OpaqueFunction(
        function=launch_runtime_nodes,
        args=[moveit_config, move_group_parameters],
    )

    launch_joy_arg = DeclareLaunchArgument('launch_joy', default_value='false')
    launch_moveit_iface_arg = DeclareLaunchArgument(
        'launch_moveit_iface',
        default_value='true',
        description='Launch the embedded arm_api2 moveit2 interface',
    )
    launch_servo_watchdog_arg = DeclareLaunchArgument(
        'launch_servo_watchdog',
        default_value='true',
        description='Launch the arm_api2 servo watchdog node',
    )
    quiet_runtime_output_arg = DeclareLaunchArgument(
        "quiet_runtime_output",
        default_value="false",
        description="Write persistent runtime output to logs only.",
    )

    # arm_api2 moveit wrapper
    moveit_wrapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("arm_api2"), 
                'launch',
                'moveit2_iface.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': 'ur',
            'launch_joy': LaunchConfiguration("launch_joy"),
            'launch_servo_watchdog': LaunchConfiguration(
                "launch_servo_watchdog"
            ),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_moveit_iface")),
    )



    return LaunchDescription([
        rviz_config_arg,
        launch_rviz_arg,
        launch_joy_arg,
        launch_moveit_iface_arg,
        launch_servo_watchdog_arg,
        quiet_runtime_output_arg,
        runtime_nodes,
        moveit_wrapper,
    ])

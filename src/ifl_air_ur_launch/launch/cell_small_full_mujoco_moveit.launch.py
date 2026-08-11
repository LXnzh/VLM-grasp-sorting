from pathlib import Path
import shlex

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
import yaml


SCENE_MODES = ("mix", "random", "assign")
MAX_ASSIGNED_OBJECTS = 6
ASSIGNED_OBJECTS_PROMPT_SENTINEL = "prompt"
DEFAULT_SIM_CONFIG_PATH = Path(
    "/home/ws/src/ifl_air_mujoco_sim/env/config/base_env.yaml"
)


def resolve_runtime_output(quiet_requested, normal_output):
    """Select log-only output for an explicitly quiet runtime."""
    quiet = str(quiet_requested).strip().casefold()
    if quiet in {"1", "true", "yes", "on"}:
        return {"both": "log"}
    return normal_output


def resolve_sim_headless(requested_value):
    """Normalize the explicit MuJoCo display-mode launch argument."""
    normalized_value = str(requested_value).strip().casefold()
    if normalized_value in {"1", "true", "yes", "on"}:
        return "true"
    if normalized_value in {"0", "false", "no", "off"}:
        return "false"
    raise ValueError(
        "sim_headless must be true or false; "
        f"got {requested_value!r}"
    )


def resolve_scene_mode(requested_mode, input_fn=None, output_fn=None):
    """Resolve an explicit mode or interactively prompt until one is valid."""
    input_fn = input if input_fn is None else input_fn
    output_fn = print if output_fn is None else output_fn
    normalized_mode = str(requested_mode).strip().lower()

    if normalized_mode != "prompt":
        if normalized_mode not in SCENE_MODES:
            raise ValueError(
                "scene_mode must be one of: mix, random, assign; "
                f"got {requested_mode!r}"
            )
        return normalized_mode

    while True:
        try:
            response = input_fn("Select scene mode [mix/random/assign]: ")
        except EOFError as exc:
            raise RuntimeError(
                "scene mode input stream closed before a selection was made"
            ) from exc

        normalized_response = str(response).strip().lower()
        if normalized_response in SCENE_MODES:
            return normalized_response
        output_fn("Invalid scene mode. Enter 'mix', 'random', or 'assign'.")


def load_available_object_names(config_path=DEFAULT_SIM_CONFIG_PATH):
    """Load and validate canonical object names from the simulator YAML."""
    config_path = Path(config_path)
    try:
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(
            f"could not load simulator config {config_path}: {exc}"
        ) from exc

    if not isinstance(config, dict) or "objects" not in config:
        raise ValueError("simulator config must contain an objects list")
    objects = config["objects"]
    if not isinstance(objects, list) or not objects:
        raise ValueError("simulator config objects must be a non-empty list")

    names = []
    normalized_names = set()
    for index, obj in enumerate(objects, start=1):
        name = obj.get("name") if isinstance(obj, dict) else None
        if not isinstance(name, str) or not name.strip():
            raise ValueError(
                f"simulator object entry {index} must have a valid name"
            )
        canonical_name = name.strip()
        normalized_name = canonical_name.casefold()
        if normalized_name in normalized_names:
            raise ValueError(f"duplicate object name: {normalized_name}")
        names.append(canonical_name)
        normalized_names.add(normalized_name)
    return tuple(names)


def parse_assigned_object_names(
    raw_names,
    available_names,
    max_count=MAX_ASSIGNED_OBJECTS,
):
    """Parse one comma-separated assignment line into canonical names."""
    text = str(raw_names).strip()
    if not text:
        return []

    tokens = [token.strip() for token in text.split(",")]
    if any(not token for token in tokens):
        raise ValueError("empty entry between commas")
    if len(tokens) > max_count:
        raise ValueError(
            f"enter at most {max_count} objects; got {len(tokens)}"
        )

    canonical_by_normalized_name = {
        str(name).strip().casefold(): str(name).strip()
        for name in available_names
    }
    parsed_names = []
    seen_names = set()
    for token in tokens:
        normalized_name = token.casefold()
        if normalized_name not in canonical_by_normalized_name:
            raise ValueError(f"unknown object: {token}")
        canonical_name = canonical_by_normalized_name[normalized_name]
        if normalized_name in seen_names:
            raise ValueError(f"duplicate object: {canonical_name}")
        parsed_names.append(canonical_name)
        seen_names.add(normalized_name)
    return parsed_names


def resolve_assigned_object_names(
    available_names,
    input_fn=None,
    output_fn=None,
):
    """Prompt until a complete assigned-object line passes validation."""
    input_fn = input if input_fn is None else input_fn
    output_fn = print if output_fn is None else output_fn
    available_names = tuple(available_names)

    while True:
        output_fn(
            f"Available objects ({len(available_names)}): "
            + ", ".join(available_names)
        )
        try:
            response = input_fn(
                "Enter up to 6 required objects, comma-separated "
                "(blank = all random): "
            )
        except EOFError as exc:
            raise RuntimeError(
                "assigned-object input stream closed before a selection was made"
            ) from exc

        try:
            return parse_assigned_object_names(response, available_names)
        except ValueError as exc:
            output_fn(f"Invalid assigned objects: {exc}")


def resolve_assigned_object_argument(
    raw_names,
    available_names,
    input_fn=None,
    output_fn=None,
):
    """Resolve the interactive sentinel or validate an explicit list."""
    raw_text = str(raw_names).strip()
    if raw_text.casefold() == ASSIGNED_OBJECTS_PROMPT_SENTINEL:
        return resolve_assigned_object_names(
            available_names,
            input_fn=input_fn,
            output_fn=output_fn,
        )

    if raw_text.startswith("[") and raw_text.endswith("]"):
        raw_text = raw_text[1:-1].strip()
    return parse_assigned_object_names(raw_text, available_names)


def launch_mujoco_ros_interface(context):
    requested_mode = context.perform_substitution(
        LaunchConfiguration("scene_mode")
    )
    scene_mode = resolve_scene_mode(requested_mode)
    print(f"[INFO] Selected scene mode: {scene_mode}")

    hydra_overrides = [f"scene_mode={scene_mode}"]
    if scene_mode == "assign":
        available_names = load_available_object_names()
        assigned_argument = context.perform_substitution(
            LaunchConfiguration("assigned_object_names")
        )
        assigned_names = resolve_assigned_object_argument(
            assigned_argument,
            available_names,
        )
        assigned_value = ",".join(assigned_names)
        hydra_overrides.append(
            f"assigned_object_names=[{assigned_value}]"
        )
    headless_requested = context.perform_substitution(
        LaunchConfiguration("sim_headless")
    )
    sim_headless = resolve_sim_headless(headless_requested)
    hydra_overrides.append(f"sim.headless={sim_headless}")
    override_command = " ".join(
        shlex.quote(override) for override in hydra_overrides
    )
    render_environment = "MUJOCO_GL=egl " if sim_headless == "true" else ""
    quiet_requested = context.perform_substitution(
        LaunchConfiguration("quiet_runtime_output")
    )

    return [
        ExecuteProcess(
            cmd=[
                "bash",
                "-lc",
                (
                    "cd /home/ws && "
                    "source /opt/ros/humble/setup.bash && "
                    "source install/setup.bash && "
                    "cd src/ifl_air_mujoco_sim && "
                    f"{render_environment}python3 ros2_main.py "
                    f"{override_command}"
                ),
            ],
            name="ifl_air_mujoco_ros_interface",
            output=resolve_runtime_output(quiet_requested, "screen"),
        )
    ]


def generate_launch_description():
    launch_robot_rviz_arg = DeclareLaunchArgument(
        "launch_robot_rviz",
        default_value="false",
        description="Launch RViz from the robot_state_publisher/gripper launch.",
    )
    launch_moveit_rviz_arg = DeclareLaunchArgument(
        "launch_moveit_rviz",
        default_value="true",
        description="Launch RViz from the MoveIt launch.",
    )
    launch_joy_arg = DeclareLaunchArgument(
        "launch_joy",
        default_value="false",
        description="Launch joystick support in arm_api2 MoveIt wrapper.",
    )
    launch_moveit_iface_arg = DeclareLaunchArgument(
        "launch_moveit_iface",
        default_value="true",
        description="Launch the embedded arm_api2 moveit2 interface.",
    )
    launch_servo_watchdog_arg = DeclareLaunchArgument(
        "launch_servo_watchdog",
        default_value="true",
        description="Launch the arm_api2 servo watchdog with the interface.",
    )
    quiet_runtime_output_arg = DeclareLaunchArgument(
        "quiet_runtime_output",
        default_value="false",
        description="Write persistent runtime output to logs only.",
    )
    sim_headless_arg = DeclareLaunchArgument(
        "sim_headless",
        default_value="false",
        description=(
            "Run MuJoCo without its GLFW viewer. Keep false for the normal "
            "interactive GUI and use true on a display-less host."
        ),
    )
    scene_mode_arg = DeclareLaunchArgument(
        "scene_mode",
        default_value="prompt",
        description=(
            "Scene selection mode: prompt interactively, preserve the current "
            "mix scene, sample one object per category with random, or require "
            "specific objects with assign."
        ),
    )
    assigned_object_names_arg = DeclareLaunchArgument(
        "assigned_object_names",
        default_value=ASSIGNED_OBJECTS_PROMPT_SENTINEL,
        description=(
            "Assigned object list for scene_mode=assign. Use prompt for "
            "interactive input or [] for an explicitly all-random scene."
        ),
    )

    mujoco_ros_interface = OpaqueFunction(
        function=launch_mujoco_ros_interface,
    )

    robot_state_and_gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ifl_air_ur_launch"),
                    "launch",
                    "cell_small_ur_orbbec_robotiq_mujoco.launch.py",
                ]
            )
        ),
        launch_arguments={
            "launch_rviz": LaunchConfiguration("launch_robot_rviz"),
            "quiet_runtime_output": LaunchConfiguration(
                "quiet_runtime_output"
            ),
        }.items(),
    )

    moveit_and_arm_api = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ifl_air_ur_launch"),
                    "launch",
                    "moveit_cell_small_ur_orbbec_robotiq.launch.py",
                ]
            )
        ),
        launch_arguments={
            "launch_rviz": LaunchConfiguration("launch_moveit_rviz"),
            "launch_joy": LaunchConfiguration("launch_joy"),
            "launch_moveit_iface": LaunchConfiguration("launch_moveit_iface"),
            "launch_servo_watchdog": LaunchConfiguration(
                "launch_servo_watchdog"
            ),
            "quiet_runtime_output": LaunchConfiguration(
                "quiet_runtime_output"
            ),
        }.items(),
    )

    return LaunchDescription(
        [
            launch_robot_rviz_arg,
            launch_moveit_rviz_arg,
            launch_joy_arg,
            launch_moveit_iface_arg,
            launch_servo_watchdog_arg,
            quiet_runtime_output_arg,
            sim_headless_arg,
            scene_mode_arg,
            assigned_object_names_arg,
            mujoco_ros_interface,
            TimerAction(period=5.0, actions=[robot_state_and_gripper]),
            moveit_and_arm_api,
        ]
    )

import getpass
import importlib.util
from pathlib import Path
import sys
import threading

from launch import LaunchDescription
from launch.actions import (
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


STALE_SESSION_ENVIRONMENT_KEYS = (
    "ROS_DOMAIN_ID",
    "VLM_CANDIDATE_OVERRIDE",
    "FOUNDATIONPOSE_MASK_INDEX",
)


def _load_full_launch_module():
    path = Path(__file__).with_name(
        "cell_small_full_mujoco_moveit.launch.py"
    )
    spec = importlib.util.spec_from_file_location(
        "ifl_air_full_launch_session_helpers",
        path,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_FULL_LAUNCH = _load_full_launch_module()
load_available_object_names = _FULL_LAUNCH.load_available_object_names
resolve_assigned_object_names = _FULL_LAUNCH.resolve_assigned_object_names


def resolve_vlm_api_key(environ, getpass_fn=None):
    """Reuse an exported key or securely prompt until a non-empty key exists."""
    getpass_fn = getpass.getpass if getpass_fn is None else getpass_fn
    inherited = str(environ.get("VLM_API_KEY", "")).strip()
    if inherited:
        return inherited

    while True:
        try:
            entered = str(getpass_fn("VLM API key (hidden): ")).strip()
        except EOFError as exc:
            raise RuntimeError(
                "API-key input stream closed before a value was provided"
            ) from exc
        if entered:
            return entered
        print("VLM API key cannot be empty.")


def build_session_environment(environ, api_key):
    """Build a child environment without stale domain/selection overrides."""
    child_env = dict(environ)
    for key in STALE_SESSION_ENVIRONMENT_KEYS:
        child_env.pop(key, None)
    child_env["VLM_API_KEY"] = str(api_key)
    return child_env


def write_process_stdin(process_action, data):
    """Write bytes to Humble's private subprocess stdin transport."""
    subprocess_transport = getattr(
        process_action,
        "_subprocess_transport",
        None,
    )
    if subprocess_transport is None:
        raise RuntimeError("supervisor stdin transport is unavailable")
    stdin_transport = subprocess_transport.get_pipe_transport(0)
    if stdin_transport is None or stdin_transport.is_closing():
        raise RuntimeError("supervisor stdin transport is unavailable")
    stdin_transport.write(data)


def _schedule_stdin_write(context, process_event, data):
    # Humble's ProcessStdin handler only logs a warning and does not write.
    context.asyncio_loop.call_soon_threadsafe(
        write_process_stdin,
        process_event.action,
        data,
    )


def start_stdin_forwarder(context, process_event, input_stream=None):
    """Forward terminal bytes to an ExecuteProcess stdin pipe."""
    input_stream = sys.stdin.buffer if input_stream is None else input_stream

    def _pump():
        while True:
            data = input_stream.readline()
            if isinstance(data, str):
                data = data.encode()
            if not data:
                _schedule_stdin_write(context, process_event, b"q\n")
                return
            _schedule_stdin_write(context, process_event, data)

    thread = threading.Thread(
        target=_pump,
        name="experiment-session-stdin",
        daemon=True,
    )
    thread.start()
    return thread


def build_session_actions(
    context,
    *,
    getpass_fn=None,
    input_fn=None,
    output_fn=None,
):
    """Resolve startup interaction before returning any launch children."""
    api_key = resolve_vlm_api_key(
        context.environment,
        getpass_fn=getpass_fn,
    )
    child_env = build_session_environment(context.environment, api_key)
    context.environment.clear()
    context.environment.update(child_env)

    available_names = load_available_object_names()
    assigned_names = resolve_assigned_object_names(
        available_names,
        input_fn=input_fn,
        output_fn=output_fn,
    )
    assigned_argument = "[" + ",".join(assigned_names) + "]"

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ifl_air_ur_launch"),
                    "launch",
                    "cell_small_full_mujoco_moveit.launch.py",
                ]
            )
        ),
        launch_arguments={
            "scene_mode": "assign",
            "assigned_object_names": assigned_argument,
            "quiet_runtime_output": "true",
            "launch_moveit_iface": "false",
            "launch_robot_rviz": "false",
            "launch_moveit_rviz": "false",
            "launch_joy": "false",
            "launch_servo_watchdog": "false",
        }.items(),
    )

    supervisor = ExecuteProcess(
        cmd=["ros2", "run", "my_course_pkg", "experiment_session"],
        name="experiment_session_supervisor",
        output="screen",
        emulate_tty=True,
    )

    def _on_supervisor_start(event, launch_context):
        start_stdin_forwarder(launch_context, event)

    stdin_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=supervisor,
            on_start=_on_supervisor_start,
        )
    )
    exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=supervisor,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="experiment session supervisor exited"
                    )
                )
            ],
        )
    )
    return [base_launch, stdin_handler, exit_handler, supervisor]


def generate_launch_description():
    return LaunchDescription(
        [OpaqueFunction(function=build_session_actions)]
    )

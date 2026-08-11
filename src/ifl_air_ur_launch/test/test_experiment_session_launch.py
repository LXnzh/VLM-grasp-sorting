import importlib.util
import io
from pathlib import Path
from types import SimpleNamespace


LAUNCH_DIR = Path(__file__).resolve().parents[1] / "launch"


def _load_launch_module(filename, module_name):
    path = LAUNCH_DIR / filename
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _declared_argument(module, name):
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, module.DeclareLaunchArgument) and entity.name == name:
            return entity
    raise AssertionError(f"launch argument {name!r} was not declared")


def _argument_default_text(argument):
    return "".join(substitution.text for substitution in argument.default_value)


def test_key_reuses_environment_without_prompt():
    module = _load_launch_module(
        "experiment_session.launch.py",
        "experiment_session_launch_key_env",
    )

    def fail_if_prompted(_prompt):
        raise AssertionError("exported API key must bypass hidden prompt")

    assert module.resolve_vlm_api_key(
        {"VLM_API_KEY": "session-secret"},
        getpass_fn=fail_if_prompted,
    ) == "session-secret"


def test_missing_key_uses_hidden_prompt_and_retries_blank():
    module = _load_launch_module(
        "experiment_session.launch.py",
        "experiment_session_launch_key_prompt",
    )
    responses = iter(["  ", "session-secret"])
    prompts = []

    result = module.resolve_vlm_api_key(
        {},
        getpass_fn=lambda prompt: prompts.append(prompt) or next(responses),
    )

    assert result == "session-secret"
    assert len(prompts) == 2


def test_session_environment_removes_stale_overrides():
    module = _load_launch_module(
        "experiment_session.launch.py",
        "experiment_session_launch_environment",
    )
    original = {
        "VLM_API_KEY": "old",
        "ROS_DOMAIN_ID": "41",
        "VLM_CANDIDATE_OVERRIDE": "banana",
        "FOUNDATIONPOSE_MASK_INDEX": "2",
        "KEEP_ME": "yes",
    }

    child_env = module.build_session_environment(original, "session-secret")

    assert child_env["VLM_API_KEY"] == "session-secret"
    assert child_env["KEEP_ME"] == "yes"
    assert "ROS_DOMAIN_ID" not in child_env
    assert "VLM_CANDIDATE_OVERRIDE" not in child_env
    assert "FOUNDATIONPOSE_MASK_INDEX" not in child_env
    assert original["ROS_DOMAIN_ID"] == "41"


def test_full_launch_defaults_to_embedded_interface_and_watchdog():
    module = _load_launch_module(
        "cell_small_full_mujoco_moveit.launch.py",
        "full_launch_interface_defaults",
    )

    interface_arg = _declared_argument(module, "launch_moveit_iface")
    watchdog_arg = _declared_argument(module, "launch_servo_watchdog")
    quiet_output_arg = _declared_argument(module, "quiet_runtime_output")
    robot_rviz_arg = _declared_argument(module, "launch_robot_rviz")
    moveit_rviz_arg = _declared_argument(module, "launch_moveit_rviz")

    assert _argument_default_text(interface_arg).lower() == "true"
    assert _argument_default_text(watchdog_arg).lower() == "true"
    assert _argument_default_text(quiet_output_arg).lower() == "false"
    assert _argument_default_text(robot_rviz_arg).lower() == "false"
    assert _argument_default_text(moveit_rviz_arg).lower() == "true"


def test_moveit_launch_declares_conditional_interface_and_watchdog():
    module = _load_launch_module(
        "moveit_cell_small_ur_orbbec_robotiq.launch.py",
        "moveit_launch_conditional_interface",
    )

    _declared_argument(module, "launch_moveit_iface")
    _declared_argument(module, "quiet_runtime_output")
    watchdog_arg = _declared_argument(module, "launch_servo_watchdog")
    assert _argument_default_text(watchdog_arg).lower() == "true"

    includes = [
        entity
        for entity in module.generate_launch_description().entities
        if isinstance(entity, module.IncludeLaunchDescription)
    ]
    assert len(includes) == 1
    assert includes[0].condition is not None


def test_quiet_runtime_output_switches_only_requested_launches():
    for filename, module_name, normal_output in (
        (
            "cell_small_full_mujoco_moveit.launch.py",
            "full_launch_quiet_output",
            "screen",
        ),
        (
            "moveit_cell_small_ur_orbbec_robotiq.launch.py",
            "moveit_launch_quiet_output",
            "screen",
        ),
        (
            "cell_small_ur_orbbec_robotiq_mujoco.launch.py",
            "robot_launch_quiet_output",
            "both",
        ),
    ):
        module = _load_launch_module(filename, module_name)
        _declared_argument(module, "quiet_runtime_output")
        quiet_output = module.resolve_runtime_output("true", normal_output)
        assert quiet_output == {"both": "log"}
        assert "screen" not in quiet_output.values()
        assert (
            module.resolve_runtime_output("false", normal_output)
            == normal_output
        )


def test_full_launch_forwards_quiet_runtime_output_to_nested_launches():
    module = _load_launch_module(
        "cell_small_full_mujoco_moveit.launch.py",
        "full_launch_quiet_output_forwarding",
    )

    includes = []
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, module.IncludeLaunchDescription):
            includes.append(entity)
        if isinstance(entity, module.TimerAction):
            includes.extend(
                action
                for action in entity.actions
                if isinstance(action, module.IncludeLaunchDescription)
            )

    assert len(includes) == 2
    for include in includes:
        forwarded = dict(include.launch_arguments)
        assert "quiet_runtime_output" in forwarded
        assert "quiet_runtime_output" in forwarded[
            "quiet_runtime_output"
        ].describe()


def test_session_launch_resolves_startup_before_returning_children(monkeypatch):
    module = _load_launch_module(
        "experiment_session.launch.py",
        "experiment_session_launch_actions",
    )
    monkeypatch.setattr(
        module,
        "load_available_object_names",
        lambda: tuple(f"object_{index}" for index in range(18)),
    )
    monkeypatch.setattr(
        module,
        "resolve_assigned_object_names",
        lambda names, **_kwargs: [names[0], names[1]],
    )

    class FakeContext:
        environment = {
            "VLM_API_KEY": "session-secret",
            "ROS_DOMAIN_ID": "41",
        }

    actions = module.build_session_actions(
        FakeContext(),
        getpass_fn=lambda _prompt: (_ for _ in ()).throw(
            AssertionError("exported key must not prompt")
        ),
        input_fn=lambda _prompt: "object_0,object_1",
        output_fn=lambda _message: None,
    )

    assert len(actions) == 4
    assert isinstance(actions[0], module.IncludeLaunchDescription)
    assert isinstance(actions[1], module.RegisterEventHandler)
    assert isinstance(actions[2], module.RegisterEventHandler)
    assert isinstance(actions[3], module.ExecuteProcess)
    rendered = repr(actions)
    assert "session-secret" not in rendered
    launch_arguments = dict(actions[0].launch_arguments)
    assert launch_arguments["launch_moveit_iface"] == "false"
    assert launch_arguments["launch_robot_rviz"] == "false"
    assert launch_arguments["launch_moveit_rviz"] == "false"
    assert launch_arguments["quiet_runtime_output"] == "true"
    assert launch_arguments["assigned_object_names"] == "[object_0,object_1]"
    assert "ROS_DOMAIN_ID" not in FakeContext.environment
    assert FakeContext.environment["VLM_API_KEY"] == "session-secret"


def test_session_launch_description_has_one_early_setup_action():
    module = _load_launch_module(
        "experiment_session.launch.py",
        "experiment_session_launch_description",
    )

    entities = list(module.generate_launch_description().entities)

    assert len(entities) == 1
    assert isinstance(entities[0], module.OpaqueFunction)


def test_stdin_forwarder_sends_terminal_lines_and_eof_quit():
    module = _load_launch_module(
        "experiment_session.launch.py",
        "experiment_session_stdin_forwarder",
    )
    events = []

    class ImmediateLoop:
        @staticmethod
        def call_soon_threadsafe(callback, *args):
            callback(*args)

    class FakeStdinTransport:
        @staticmethod
        def is_closing():
            return False

        @staticmethod
        def write(data):
            events.append(data)

    class FakeSubprocessTransport:
        @staticmethod
        def get_pipe_transport(index):
            assert index == 0
            return FakeStdinTransport()

    class FakeContext:
        asyncio_loop = ImmediateLoop()

    process_action = SimpleNamespace(
        _subprocess_transport=FakeSubprocessTransport(),
    )
    process_event = SimpleNamespace(
        action=process_action,
        process_name="experiment_session_supervisor",
        cmd=["ros2", "run", "my_course_pkg", "experiment_session"],
        cwd=None,
        env={},
        pid=123,
    )
    thread = module.start_stdin_forwarder(
        FakeContext(),
        process_event,
        input_stream=io.BytesIO(b"pick up the apple\n\n"),
    )
    thread.join(timeout=2.0)

    assert events == [
        b"pick up the apple\n",
        b"\n",
        b"q\n",
    ]


def test_direct_stdin_write_fails_if_process_pipe_is_unavailable():
    module = _load_launch_module(
        "experiment_session.launch.py",
        "experiment_session_missing_stdin_pipe",
    )
    process_action = SimpleNamespace(_subprocess_transport=None)

    try:
        module.write_process_stdin(process_action, b"instruction\n")
    except RuntimeError as exc:
        assert "stdin transport is unavailable" in str(exc)
    else:
        raise AssertionError("missing supervisor stdin pipe must fail closed")

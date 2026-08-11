import io
import os
import signal
import subprocess
import sys

from my_course_pkg import experiment_session as session


def _ok(stage, output=""):
    return session.StageResult(stage=stage, returncode=0, output=output)


def _failed(stage, message="failed"):
    return session.StageResult(stage=stage, returncode=1, message=message)


class FakeInterface:
    def __init__(self, events, start_results=None, stop_results=None):
        self.events = events
        self.owned = False
        self.start_results = iter(start_results or [])
        self.stop_results = iter(stop_results or [])

    def has_owned_process(self):
        return self.owned

    def start(self, trial):
        self.events.append(("start_interface", trial))
        result = next(self.start_results, _ok("start_interface"))
        if result.ok:
            self.owned = True
        return result

    def stop(self, trial):
        self.events.append(("stop_interface", trial))
        result = next(self.stop_results, _ok("stop_interface"))
        self.owned = False
        return result

    def cleanup(self):
        self.events.append(("cleanup", None))
        self.owned = False
        return _ok("cleanup")


class FakeProcessProbe:
    def __init__(self, events, results=None):
        self.events = events
        self.results = iter(results or [])

    def verify_empty(self, trial, log_path):
        self.events.append(("verify_empty", trial))
        return next(self.results, _ok("verify_empty"))


class FakeGraphProbe:
    def __init__(self, events, results=None):
        self.events = events
        self.results = iter(results or [])

    def wait_until_ready(self, trial, timeout_sec, log_path):
        self.events.append(("graph_ready", trial))
        return next(self.results, _ok("graph_ready"))


class FakeCommandRunner:
    def __init__(self, events, results=None):
        self.events = events
        self.results = dict(results or {})

    def run(
        self,
        stage,
        command,
        *,
        timeout_sec,
        env,
        input_text=None,
        log_path,
    ):
        self.events.append((stage, input_text))
        configured = self.results.get(stage)
        if configured is not None:
            return configured
        output = "success=True" if stage == "reset_sim" else ""
        return _ok(stage, output=output)


class FakeClock:
    def __init__(self):
        self.now = 0.0
        self.sleeps = []

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        seconds = float(seconds)
        self.sleeps.append(seconds)
        self.now += seconds


def _graph_run_sequence(snapshots, calls=None):
    snapshots = list(snapshots)
    command_count = 0

    def run_fn(command, **kwargs):
        nonlocal command_count
        snapshot_index = min(command_count // 4, len(snapshots) - 1)
        snapshot = snapshots[snapshot_index]
        command_count += 1
        if calls is not None:
            calls.append((list(command), kwargs))

        rendered = " ".join(command)
        if "node list" in rendered:
            stdout = "\n".join(session.REQUIRED_NODE_NAMES) + "\n"
        elif "topic info" in rendered:
            publisher_count = 0 if snapshot == "missing" else 1
            stdout = f"Publisher count: {publisher_count}\n"
        elif "/arm/move_to_pose" in rendered:
            server_count = {
                "ready": 1,
                "missing": 0,
                "duplicate": 2,
            }[snapshot]
            stdout = f"Action servers: {server_count}\n"
        else:
            stdout = "Action servers: 1\n"
        return subprocess.CompletedProcess(command, 0, stdout, "")

    return run_fn


def _make_controller(tmp_path, responses, *, command_results=None, probe_results=None):
    events = []
    controller = session.ExperimentSession(
        config=session.SessionConfig(log_root=tmp_path, settle_sec=0.0),
        interface_manager=FakeInterface(events),
        process_probe=FakeProcessProbe(events, probe_results),
        graph_probe=FakeGraphProbe(events),
        command_runner=FakeCommandRunner(events, command_results),
        input_fn=lambda _prompt: next(responses),
        output_fn=lambda _message: None,
        sleep_fn=lambda _seconds: events.append(("settle", None)),
        environ={"VLM_API_KEY": "secret", "ROS_DOMAIN_ID": "41"},
    )
    return controller, events


def test_session_environment_keeps_key_and_removes_manual_selection_overrides():
    result = session.build_trial_environment(
        {
            "VLM_API_KEY": "secret",
            "ROS_DOMAIN_ID": "41",
            "VLM_CANDIDATE_OVERRIDE": "banana",
            "FOUNDATIONPOSE_MASK_INDEX": "2",
            "GRASP_DEBUG_STOP_AFTER_LIFT": "1",
        }
    )

    assert result["VLM_API_KEY"] == "secret"
    assert result["GRASP_DEBUG_STOP_AFTER_LIFT"] == "1"
    assert "ROS_DOMAIN_ID" not in result
    assert "VLM_CANDIDATE_OVERRIDE" not in result
    assert "FOUNDATIONPOSE_MASK_INDEX" not in result


def test_first_trial_runs_exact_safe_order_and_pipeline_feeds_instruction(tmp_path):
    controller, events = _make_controller(
        tmp_path,
        iter(["pick up the blue racquetball", "q"]),
    )

    assert controller.run() == 0
    assert events == [
        ("verify_empty", 1),
        ("start_interface", 1),
        ("graph_ready", 1),
        ("reset_sim", None),
        ("settle", None),
        ("pipeline", "pick up the blue racquetball\n"),
        ("grasp_demo", None),
        ("cleanup", None),
    ]


def test_enter_after_success_restarts_from_owned_interface_boundary(tmp_path):
    controller, events = _make_controller(
        tmp_path,
        iter(["pick up the banana", "", "pick up the apple", "q"]),
    )

    assert controller.run() == 0
    assert ("stop_interface", 2) in events
    second_stop = events.index(("stop_interface", 2))
    assert events[second_stop:second_stop + 4] == [
        ("stop_interface", 2),
        ("verify_empty", 2),
        ("start_interface", 2),
        ("graph_ready", 2),
    ]


def test_blank_instruction_reprompts_without_running_pipeline(tmp_path):
    controller, events = _make_controller(
        tmp_path,
        iter(["  ", "pick up the hammer", "q"]),
    )

    assert controller.run() == 0
    pipeline_events = [event for event in events if event[0] == "pipeline"]
    assert pipeline_events == [("pipeline", "pick up the hammer\n")]


def test_interactive_prompts_are_printed_as_complete_lines(tmp_path):
    events = []
    responses = iter(["pick up the apple", "q"])
    input_prompts = []
    output_lines = []
    controller = session.ExperimentSession(
        config=session.SessionConfig(log_root=tmp_path, settle_sec=0.0),
        interface_manager=FakeInterface(events),
        process_probe=FakeProcessProbe(events),
        graph_probe=FakeGraphProbe(events),
        command_runner=FakeCommandRunner(events),
        input_fn=lambda prompt: input_prompts.append(prompt) or next(responses),
        output_fn=output_lines.append,
        sleep_fn=lambda _seconds: None,
        environ={"VLM_API_KEY": "secret"},
    )

    assert controller.run() == 0
    assert input_prompts == ["", ""]
    assert "Instruction:" in output_lines
    assert "Press Enter for the next trial, or q to quit:" in output_lines


def test_pipeline_failure_skips_grasp_and_q_returns_failure(tmp_path):
    controller, events = _make_controller(
        tmp_path,
        iter(["pick up the apple", "q"]),
        command_results={"pipeline": _failed("pipeline")},
    )

    assert controller.run() == 1
    assert not any(event[0] == "grasp_demo" for event in events)
    assert events[-1] == ("cleanup", None)


def test_reset_success_false_stops_before_instruction(tmp_path):
    controller, events = _make_controller(
        tmp_path,
        iter(["q"]),
        command_results={"reset_sim": _ok("reset_sim", "success=False")},
    )

    assert controller.run() == 1
    assert not any(event[0] == "pipeline" for event in events)


def test_enter_after_failure_retries_complete_boundary(tmp_path):
    controller, events = _make_controller(
        tmp_path,
        iter(["", "pick up the apple", "q"]),
        probe_results=[_failed("verify_empty", "stale process"), _ok("verify_empty")],
    )

    assert controller.run() == 0
    assert events[:3] == [
        ("verify_empty", 1),
        ("verify_empty", 2),
        ("start_interface", 2),
    ]


def test_foreign_process_failure_never_starts_interface(tmp_path):
    controller, events = _make_controller(
        tmp_path,
        iter(["q"]),
        probe_results=[_failed("verify_empty", "123 moveit2_iface")],
    )

    assert controller.run() == 1
    assert not any(event[0] == "start_interface" for event in events)


def test_trigger_success_parser_requires_true_response():
    assert session.trigger_response_succeeded("success=True\nmessage='ok'") is True
    assert session.trigger_response_succeeded("success: true\nmessage: ok") is True
    assert session.trigger_response_succeeded("success=False") is False
    assert session.trigger_response_succeeded("") is False


def test_ros_graph_snapshot_detects_ready_missing_and_duplicates():
    ready = session.evaluate_graph_snapshot(
        node_names=[
            "/moveit2_iface",
            "/robotiq_2f_urcap_adapter",
            "/robot_state_publisher",
            "/move_group",
        ],
        arm_action_servers=1,
        gripper_action_servers=1,
        pose_publishers=1,
    )
    assert ready.ready is True
    assert ready.duplicate is False

    missing = session.evaluate_graph_snapshot(
        node_names=["/move_group"],
        arm_action_servers=0,
        gripper_action_servers=1,
        pose_publishers=0,
    )
    assert missing.ready is False
    assert missing.duplicate is False

    duplicate = session.evaluate_graph_snapshot(
        node_names=[
            "/moveit2_iface",
            "/moveit2_iface",
            "/robotiq_2f_urcap_adapter",
            "/robot_state_publisher",
            "/move_group",
        ],
        arm_action_servers=2,
        gripper_action_servers=1,
        pose_publishers=1,
    )
    assert duplicate.ready is False
    assert duplicate.duplicate is True


def test_ros_cli_count_parsers():
    assert session.parse_action_server_count("Action servers: 1\n") == 1
    assert session.parse_topic_publisher_count("Publisher count: 1\n") == 1
    assert session.parse_node_names("/move_group\n/moveit2_iface\n") == [
        "/move_group",
        "/moveit2_iface",
    ]


def test_graph_probe_uses_configured_cli_query_timeout(tmp_path):
    calls = []
    clock = FakeClock()

    probe = session.RosGraphProbe(
        run_fn=_graph_run_sequence(["ready", "ready"], calls),
        sleep_fn=clock.sleep,
        monotonic_fn=clock.monotonic,
        query_timeout_sec=17.0,
    )

    result = probe.wait_until_ready(1, 30.0, tmp_path / "graph.log")

    assert result.ok is True
    assert [kwargs["timeout"] for _command, kwargs in calls] == [17.0] * 8


def test_graph_probe_allows_transient_duplicate_then_requires_two_clean_samples(
    tmp_path,
):
    calls = []
    clock = FakeClock()
    probe = session.RosGraphProbe(
        run_fn=_graph_run_sequence(
            ["duplicate", "ready", "ready"],
            calls,
        ),
        sleep_fn=clock.sleep,
        monotonic_fn=clock.monotonic,
        poll_sec=0.5,
        duplicate_settle_sec=30.0,
        required_clean_samples=2,
    )

    log_path = tmp_path / "graph.log"
    result = probe.wait_until_ready(1, 60.0, log_path)

    assert result.ok is True
    assert len(calls) == 12
    assert clock.sleeps == [0.5, 0.5]
    log_text = log_path.read_text(encoding="utf-8")
    assert "duplicate settle window started" in log_text
    assert "clean ROS graph sample 1/2" in log_text


def test_graph_probe_duplicate_resets_clean_sample_streak(tmp_path):
    calls = []
    clock = FakeClock()
    probe = session.RosGraphProbe(
        run_fn=_graph_run_sequence(
            ["ready", "duplicate", "ready", "ready"],
            calls,
        ),
        sleep_fn=clock.sleep,
        monotonic_fn=clock.monotonic,
        poll_sec=0.5,
        duplicate_settle_sec=30.0,
        required_clean_samples=2,
    )

    result = probe.wait_until_ready(1, 60.0, tmp_path / "graph.log")

    assert result.ok is True
    assert len(calls) == 16
    assert clock.sleeps == [0.5, 0.5, 0.5]


def test_graph_probe_fails_when_duplicate_persists_through_settle_window(
    tmp_path,
):
    clock = FakeClock()
    probe = session.RosGraphProbe(
        run_fn=_graph_run_sequence(["duplicate"]),
        sleep_fn=clock.sleep,
        monotonic_fn=clock.monotonic,
        poll_sec=0.5,
        duplicate_settle_sec=1.0,
        required_clean_samples=2,
    )

    result = probe.wait_until_ready(1, 60.0, tmp_path / "graph.log")

    assert result.ok is False
    assert result.timed_out is False
    assert "duplicate ROS graph endpoints" in result.message
    assert clock.sleeps == [0.5, 0.5]


def test_graph_probe_missing_endpoints_use_overall_readiness_timeout(tmp_path):
    clock = FakeClock()
    probe = session.RosGraphProbe(
        run_fn=_graph_run_sequence(["missing"]),
        sleep_fn=clock.sleep,
        monotonic_fn=clock.monotonic,
        poll_sec=0.5,
        duplicate_settle_sec=30.0,
        required_clean_samples=2,
    )

    result = probe.wait_until_ready(1, 1.0, tmp_path / "graph.log")

    assert result.ok is False
    assert result.timed_out is True
    assert "readiness timed out" in result.message
    assert clock.sleeps == [0.5, 0.5]


def test_exact_process_probe_fails_closed_without_killing(tmp_path):
    calls = []

    def run_fn(command, **kwargs):
        calls.append((command, kwargs))
        return subprocess.CompletedProcess(command, 0, "123 moveit2_iface\n", "")

    probe = session.ExactProcessProbe(run_fn=run_fn)
    result = probe.verify_empty(1, tmp_path / "probe.log")

    assert result.ok is False
    assert "123 moveit2_iface" in result.message
    assert calls[0][0] == ["pgrep", "-a", "-x", "moveit2_iface"]


def test_tee_runner_streams_output_to_terminal_and_log(tmp_path):
    terminal = []
    runner = session.TeeCommandRunner(output_fn=terminal.append)
    log_path = tmp_path / "stage.log"

    result = runner.run(
        "test_stage",
        [sys.executable, "-c", "print('child line')"],
        timeout_sec=5.0,
        env=os.environ.copy(),
        log_path=log_path,
    )

    assert result.ok is True
    assert "child line" in result.output
    assert "child line" in log_path.read_text(encoding="utf-8")
    assert any("child line" in line for line in terminal)


class FakePopen:
    def __init__(self):
        self.pid = 123
        self.returncode = None
        self.stdout = io.StringIO("interface started\n")
        self.wait_calls = 0

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        self.wait_calls += 1
        if self.wait_calls < 3:
            raise subprocess.TimeoutExpired("interface", timeout)
        self.returncode = -signal.SIGTERM
        return self.returncode


def test_owned_interface_can_log_without_mirroring_to_terminal(tmp_path):
    process = FakePopen()
    terminal = []
    manager = session.OwnedInterfaceProcess(
        log_manager=session.SessionLogManager(tmp_path),
        popen_factory=lambda _command, **_kwargs: process,
        killpg_fn=lambda _pgid, _sig: None,
        getpgid_fn=lambda pid: pid,
        output_fn=terminal.append,
        mirror_output=False,
        stop_timeout_sec=0.01,
    )

    start_result = manager.start(1)
    assert start_result.ok is True
    manager.stop(1)

    assert terminal == []
    assert "interface started" in start_result.log_path.read_text(
        encoding="utf-8"
    )


def test_session_disables_owned_interface_output_mirroring(
    tmp_path,
    monkeypatch,
):
    captured = {}

    class CapturingInterface:
        def __init__(self, **kwargs):
            captured.update(kwargs)

    monkeypatch.setattr(session, "OwnedInterfaceProcess", CapturingInterface)

    session.ExperimentSession(
        config=session.SessionConfig(log_root=tmp_path),
        process_probe=object(),
        graph_probe=object(),
        command_runner=object(),
        input_fn=lambda _prompt: "q",
        output_fn=lambda _message: None,
        environ={"VLM_API_KEY": "secret"},
    )

    assert captured["mirror_output"] is False


def test_owned_interface_escalates_only_its_process_group(tmp_path):
    popen_calls = []
    process = FakePopen()
    signals = []

    def popen_factory(command, **kwargs):
        popen_calls.append((command, kwargs))
        return process

    manager = session.OwnedInterfaceProcess(
        log_manager=session.SessionLogManager(tmp_path),
        popen_factory=popen_factory,
        killpg_fn=lambda pgid, sig: signals.append((pgid, sig)),
        getpgid_fn=lambda pid: pid,
        output_fn=lambda _line: None,
        stop_timeout_sec=0.01,
    )

    assert manager.start(1).ok is True
    assert popen_calls[0][1]["start_new_session"] is True
    assert manager.stop(2).ok is True
    assert signals == [
        (123, signal.SIGINT),
        (123, signal.SIGTERM),
        (123, signal.SIGKILL),
    ]
    assert manager.cleanup().ok is True
    assert signals == [
        (123, signal.SIGINT),
        (123, signal.SIGTERM),
        (123, signal.SIGKILL),
    ]


def test_log_manager_creates_separate_absolute_stage_paths(tmp_path):
    logs = session.SessionLogManager(tmp_path)

    first = logs.stage_path(1, "pipeline")
    second = logs.stage_path(2, "pipeline")

    assert first.is_absolute()
    assert second.is_absolute()
    assert first != second
    assert first.parent.exists()

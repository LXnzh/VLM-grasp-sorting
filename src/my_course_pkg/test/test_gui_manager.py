"""Pure state tests for the desktop operator-guidance banner."""

import signal
import subprocess

import gui_manager
from gui_manager import (
    KEEP_STILL_GUIDANCE,
    PBVS_FOLLOWING_GUIDANCE,
    ProjectLauncher,
    TARGET_STOPPED_GUIDANCE,
    TRACKING_READY_GUIDANCE,
)


class _Value:
    def __init__(self):
        self.value = ""

    def set(self, value):
        self.value = value


class _Process:
    def __init__(self, pid, *, running=True, ignore_term=False):
        self.pid = pid
        self.running = running
        self.ignore_term = ignore_term
        self.wait_count = 0

    def poll(self):
        return None if self.running else 0

    def wait(self, timeout=None):
        self.wait_count += 1
        if self.running:
            raise subprocess.TimeoutExpired(str(self.pid), timeout)
        return 0


def _update(message, active=False):
    launcher = object.__new__(ProjectLauncher)
    launcher.task_phase = _Value()
    launcher.operator_guidance = _Value()
    if active:
        launcher.operator_guidance.set(TRACKING_READY_GUIDANCE)
    launcher.drag_window_active = active
    launcher._update_task_phase(message)
    return (
        launcher.task_phase.value,
        launcher.operator_guidance.value,
        launcher.drag_window_active,
    )


def test_target_lock_opens_visible_ten_second_drag_window():
    phase, guidance, active = _update("TARGET_LOCKED: target=banana")

    assert "10-second" in phase
    assert guidance == TRACKING_READY_GUIDANCE
    assert active is True


def test_observation_status_also_opens_drag_window():
    _, guidance, active = _update(
        "PBVS_OBSERVING_FOR_MOTION: observing for 10.00s before grasp"
    )

    assert guidance == TRACKING_READY_GUIDANCE
    assert active is True


def test_stable_gate_does_not_overwrite_active_tracking_ready_banner():
    phase, guidance, active = _update("PBVS_MOTION_GATE: state=STABLE", True)

    assert "10-second" in phase
    assert guidance == TRACKING_READY_GUIDANCE
    assert active is True


def test_follow_status_replaces_drag_prompt_and_closes_drag_window():
    _, guidance, active = _update("PBVS_FOLLOW_ACTIVE: target moving", True)

    assert guidance == PBVS_FOLLOWING_GUIDANCE
    assert active is False


def test_stop_and_execution_states_require_stationary_target():
    _, stop_guidance, stop_active = _update(
        "PBVS_WAITING_FOR_CONTINUOUS_STOP: release target", True
    )
    _, execution_guidance, execution_active = _update("EXECUTING_GRASP", True)

    assert stop_guidance == TARGET_STOPPED_GUIDANCE
    assert stop_active is False
    assert execution_guidance == KEEP_STILL_GUIDANCE
    assert execution_active is False


def test_terminal_states_replace_operator_guidance():
    _, failed_guidance, failed_active = _update(
        "TASK_FAILED: pose estimation failed", True
    )
    _, complete_guidance, complete_active = _update(
        "DONE: returned to initial pose", True
    )

    assert failed_guidance == "TASK FAILED — pose estimation failed"
    assert failed_active is False
    assert complete_guidance == (
        "TASK COMPLETE — robot returned to its initial pose."
    )
    assert complete_active is False


def test_launch_starts_each_command_in_a_new_linux_session(monkeypatch):
    process = _Process(1234)
    popen_call = {}

    def fake_popen(*args, **kwargs):
        popen_call["args"] = args
        popen_call["kwargs"] = kwargs
        return process

    class FakeThread:
        def __init__(self, **_kwargs):
            pass

        def start(self):
            pass

    monkeypatch.setattr(gui_manager.subprocess, "Popen", fake_popen)
    monkeypatch.setattr(gui_manager.threading, "Thread", FakeThread)
    launcher = object.__new__(ProjectLauncher)
    launcher._closing = False
    launcher.active_processes = {}
    launcher.status = _Value()
    launcher._process_environment = lambda **_kwargs: {}
    launcher._write_log = lambda _message: None

    launcher._launch_in_gui("task", "true", food_mode=False)

    assert popen_call["kwargs"]["start_new_session"] is True
    assert launcher.active_processes[1234] == ("task", process)


def test_process_group_shutdown_terminates_every_live_group(monkeypatch):
    first = _Process(101)
    second = _Process(202)
    exited = _Process(303, running=False)
    processes = {process.pid: process for process in (first, second, exited)}
    signals = []

    def fake_killpg(pid, sent_signal):
        signals.append((pid, sent_signal))
        process = processes[pid]
        if sent_signal == signal.SIGTERM and not process.ignore_term:
            process.running = False
        elif sent_signal == signal.SIGKILL:
            process.running = False

    monkeypatch.setattr(gui_manager.os, "killpg", fake_killpg)

    errors = gui_manager.terminate_process_groups(
        [first, second, exited],
        timeout_s=0.1,
    )

    assert errors == []
    assert signals == [(101, signal.SIGTERM), (202, signal.SIGTERM)]
    assert first.wait_count == 1
    assert second.wait_count == 1
    assert exited.wait_count == 0


def test_process_group_shutdown_force_kills_after_deadline(monkeypatch):
    process = _Process(404, ignore_term=True)
    signals = []

    def fake_killpg(_pid, sent_signal):
        signals.append(sent_signal)
        if sent_signal == signal.SIGKILL:
            process.running = False

    monkeypatch.setattr(gui_manager.os, "killpg", fake_killpg)

    errors = gui_manager.terminate_process_groups([process], timeout_s=0.0)

    assert errors == []
    assert signals == [signal.SIGTERM, signal.SIGKILL]
    assert process.wait_count == 1


def test_shutdown_is_idempotent_and_clears_owned_processes(monkeypatch):
    first = _Process(501)
    second = _Process(502)
    terminated = []
    destroyed = []

    monkeypatch.setattr(
        gui_manager,
        "terminate_process_groups",
        lambda processes: terminated.extend(processes) or [],
    )
    launcher = object.__new__(ProjectLauncher)
    launcher._closing = False
    launcher.active_processes = {
        first.pid: ("first", first),
        second.pid: ("second", second),
    }
    launcher.scene_mode = True
    launcher.destroy = lambda: destroyed.append(True)

    launcher.shutdown()
    launcher.shutdown()

    assert terminated == [first, second]
    assert launcher.active_processes == {}
    assert launcher.scene_mode is False
    assert destroyed == [True]


def test_full_stack_process_scan_matches_exact_launch_marker(tmp_path):
    matching = tmp_path / "601"
    unrelated = tmp_path / "602"
    matching.mkdir()
    unrelated.mkdir()
    (matching / "cmdline").write_bytes(
        b"python3\0ros2\0launch\0cell_small_full_mujoco_moveit.launch.py\0"
    )
    (unrelated / "cmdline").write_bytes(b"python3\0gui_manager.py\0")

    assert gui_manager.full_stack_process_ids(tmp_path) == (601,)


def _food_scene_launcher():
    launcher = object.__new__(ProjectLauncher)
    launcher.active_processes = {}
    launcher.status = _Value()
    launcher.scene_mode = None
    launcher.logs = []
    launcher.launches = []
    launcher._write_log = launcher.logs.append
    launcher.launch_terminal = (
        lambda *args, **kwargs: launcher.launches.append((args, kwargs))
    )
    return launcher


def test_external_full_stack_blocks_duplicate_scene_start(monkeypatch):
    launcher = _food_scene_launcher()
    errors = []
    monkeypatch.setattr(gui_manager, "full_stack_process_ids", lambda: (701,))
    monkeypatch.setattr(
        gui_manager.messagebox,
        "showerror",
        lambda _title, message: errors.append(message),
    )

    launcher.start_food_scene()

    assert launcher.launches == []
    assert launcher.scene_mode is None
    assert "701" in launcher.logs[-1]
    assert errors == [launcher.logs[-1]]


def test_same_gui_duplicate_guard_runs_before_external_scan(monkeypatch):
    launcher = _food_scene_launcher()
    process = _Process(801)
    launcher.active_processes[process.pid] = (
        "Food-Sorting Simulation",
        process,
    )
    monkeypatch.setattr(
        gui_manager,
        "full_stack_process_ids",
        lambda: (_ for _ in ()).throw(AssertionError("scan should not run")),
    )

    launcher.start_food_scene()

    assert launcher.launches == []
    assert "already running" in launcher.logs[-1]

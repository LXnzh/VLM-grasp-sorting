"""Pure state tests for the desktop operator-guidance banner."""

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

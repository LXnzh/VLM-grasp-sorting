from types import SimpleNamespace

import numpy as np
import pytest

from my_course_pkg.grasp import executor as executor_module
from my_course_pkg.grasp.executor import (
    ArmMotionExecutor,
    FinalApproachConvergenceError,
    GripperCommandError,
    MoveItMotionError,
    VerticalApproachCalibration,
    VerticalApproachConvergenceError,
)
from my_course_pkg.grasp.gripper_control import GripperCommandResult
from my_course_pkg.grasp.trajectory_planner import PlanStep


def gripper_result(
    *,
    accepted,
    target_position,
    actual_position,
    stalled=False,
    reached_goal=False,
    action_status=4,
):
    return GripperCommandResult(
        accepted=accepted,
        target_position=target_position,
        actual_position=actual_position,
        effort=139.7,
        stalled=stalled,
        reached_goal=reached_goal,
        action_status=action_status,
        action_succeeded=action_status == 4,
        command_mode="test",
        action_name="/fake",
        attempts=1,
    )


class DummyGripperController:
    def send_gripper_command(self, position):
        return False


class DummyLogger:
    def __init__(self):
        self.errors = []
        self.infos = []
        self.warnings = []

    def error(self, message):
        self.errors.append(message)

    def info(self, message):
        self.infos.append(message)

    def warn(self, message):
        self.warnings.append(message)


class DummyNode:
    def __init__(self):
        self.logger = DummyLogger()

    def get_logger(self):
        return self.logger


class DummyArmClient:
    def __init__(self):
        self.pose_commands = []

    def send_pose_cmd(self, pose_msg):
        self.pose_commands.append(pose_msg)


def vertical_plan_with_bounds(*, bottom_z=0.8900, center_z=0.9053):
    return SimpleNamespace(
        debug_info={
            "grasp_profile": "vertical",
            "vertical_target_bounds_center": np.array(
                [0.0, 0.0, center_z],
                dtype=float,
            ),
            "vertical_bounds_bottom_z": bottom_z,
        }
    )


def test_execute_plan_aborts_when_gripper_command_fails():
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.gripper_controller = DummyGripperController()
    plan = type(
        "Plan",
        (),
        {
            "steps": [
                PlanStep(
                    name="close_gripper_at_grasp",
                    action="gripper",
                    gripper_position=0.79,
                ),
                PlanStep(name="should_not_continue", action="hold", duration_sec=0.1),
            ]
        },
    )()

    with pytest.raises(RuntimeError, match="Gripper command failed"):
        executor.execute_plan(plan)


def test_execute_step_aborts_when_moveit_cannot_reach_pregrasp():
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.enter_moveit_mode = lambda: True
    executor.move_with_moveit = lambda pose: False

    step = PlanStep(
        name="move_to_pre_grasp",
        action="move",
        mode="moveit",
        end_pose_6d=[0.1, 0.2, 0.3, 0.0, 0.0, 0.0],
    )

    with pytest.raises(MoveItMotionError, match="move_to_pre_grasp"):
        executor.execute_step(step)


def test_execute_first_reachable_plan_tries_next_candidate_after_pregrasp_failure():
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    first_plan = object()
    second_plan = object()
    attempted = []

    def execute_plan(plan):
        attempted.append(plan)
        if plan is first_plan:
            raise MoveItMotionError("move_to_pre_grasp")

    executor.execute_plan = execute_plan

    selected = executor.execute_first_reachable_plan([first_plan, second_plan])

    assert selected is second_plan
    assert attempted == [first_plan, second_plan]
    assert len(executor.node.logger.warnings) == 1


def test_execute_first_reachable_plan_does_not_retry_after_final_convergence_failure():
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    first_plan = object()
    second_plan = object()
    attempted = []
    convergence_error = FinalApproachConvergenceError(
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.01, 0.0, 0.0, 0.0, 0.0, 0.0],
        0.01,
    )

    def execute_plan(plan):
        attempted.append(plan)
        raise convergence_error

    executor.execute_plan = execute_plan

    with pytest.raises(FinalApproachConvergenceError):
        executor.execute_first_reachable_plan([first_plan, second_plan])

    assert attempted == [first_plan]


def test_final_approach_convergence_reissues_target_until_position_is_within_tolerance(
    monkeypatch,
):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    observed_poses = iter(
        [
            np.array([0.02, 0.0, 0.0, 0.0, 0.0, 0.0]),
            np.array([0.004, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ]
    )
    executor.get_current_ee_pose_6d = lambda: next(observed_poses)
    monkeypatch.setattr(
        executor_module,
        "FINAL_APPROACH_POSITION_TOLERANCE_M",
        0.005,
    )
    monkeypatch.setattr(
        executor_module,
        "FINAL_APPROACH_SETTLE_TIMEOUT_SEC",
        1.5,
    )
    monkeypatch.setattr(
        executor_module,
        "FINAL_APPROACH_COMMAND_PERIOD_SEC",
        0.05,
    )
    monkeypatch.setattr("my_course_pkg.grasp.executor.time.sleep", lambda _: None)

    actual_pose = executor.converge_at_final_grasp(
        np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    )

    assert np.allclose(actual_pose[:3], [0.004, 0.0, 0.0])
    assert len(executor.arm_api2_client.pose_commands) == 1


def test_vertical_pregrasp_learns_recorded_command_bias(monkeypatch):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    target = np.array([0.0, 0.0, 0.100, 0.2, -0.1, 0.3])
    biased = target.copy()
    biased[:3] += np.array([0.0001, 0.0070, -0.0161])
    observed_poses = iter(
        [biased.copy() for _ in range(3)]
        + [target.copy() for _ in range(3)]
    )
    executor.get_current_ee_pose_6d = lambda: next(observed_poses)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_XY_TOLERANCE_M", 0.0015)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_COMMAND_PERIOD_SEC", 0.05)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_MAX_OFFSET_M", 0.03)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_MAX_ITERATIONS", 3)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_SETTLE_SEC", 0.4)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_SAMPLE_COUNT", 3)
    monkeypatch.setattr("my_course_pkg.grasp.executor.time.sleep", lambda _: None)

    calibration = executor.converge_at_vertical_pregrasp(target)

    assert isinstance(calibration, VerticalApproachCalibration)
    np.testing.assert_allclose(calibration.observed_median_xyz, target[:3])
    np.testing.assert_allclose(
        calibration.command_offset_xyz,
        [-0.0001, -0.0070, 0.0161],
    )
    assert calibration.command_count == 1
    assert len(executor.arm_api2_client.pose_commands) == 1
    command = executor.arm_api2_client.pose_commands[0]
    np.testing.assert_allclose(
        [command.pose.position.x, command.pose.position.y, command.pose.position.z],
        target[:3] + calibration.command_offset_xyz,
    )
    expected_orientation = executor_module._pose6d_to_posestamped_msg(
        target,
        frame_id="world",
    ).pose.orientation
    np.testing.assert_allclose(
        [
            command.pose.orientation.x,
            command.pose.orientation.y,
            command.pose.orientation.z,
            command.pose.orientation.w,
        ],
        [
            expected_orientation.x,
            expected_orientation.y,
            expected_orientation.z,
            expected_orientation.w,
        ],
    )


def test_vertical_pregrasp_keeps_strict_xy_gate_when_descent_allows_two_mm(
    monkeypatch,
):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    target = np.zeros(6)
    biased = target.copy()
    biased[0] = 0.0016
    observed_poses = iter(
        [biased.copy() for _ in range(3)]
        + [target.copy() for _ in range(3)]
    )
    executor.get_current_ee_pose_6d = lambda: next(observed_poses)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_XY_TOLERANCE_M", 0.0015)
    monkeypatch.setattr(executor_module, "VERTICAL_DESCENT_XY_TOLERANCE_M", 0.0020)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_COMMAND_PERIOD_SEC", 0.05)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_MAX_OFFSET_M", 0.03)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_MAX_ITERATIONS", 3)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_SETTLE_SEC", 0.4)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_SAMPLE_COUNT", 3)
    monkeypatch.setattr("my_course_pkg.grasp.executor.time.sleep", lambda _: None)

    calibration = executor.converge_at_vertical_pregrasp(target)

    assert calibration.command_count == 1
    np.testing.assert_allclose(calibration.command_offset_xyz, [-0.0016, 0.0, 0.0])
    assert any(
        "stage=pregrasp" in message and "xy_tolerance_m=0.0015" in message
        for message in executor.node.logger.infos
    )


def test_vertical_pregrasp_already_aligned_returns_zero_offset(monkeypatch):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    target = np.array([0.0, 0.0, 0.100, 0.0, 0.0, 0.0])
    executor.get_current_ee_pose_6d = lambda: target.copy()
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_XY_TOLERANCE_M", 0.0015)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_COMMAND_PERIOD_SEC", 0.05)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_MAX_OFFSET_M", 0.03)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_MAX_ITERATIONS", 3)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_SETTLE_SEC", 0.4)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_SAMPLE_COUNT", 3)
    monkeypatch.setattr("my_course_pkg.grasp.executor.time.sleep", lambda _: None)

    calibration = executor.converge_at_vertical_pregrasp(target)

    np.testing.assert_allclose(calibration.command_offset_xyz, np.zeros(3))
    assert calibration.command_count == 0
    assert executor.arm_api2_client.pose_commands == []


def test_vertical_feedback_stationarity_accepts_limits_inclusively(monkeypatch):
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_XY_TOLERANCE_M", 0.0015)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    sample_xyz = np.array(
        [
            [0.0, 0.0, 0.0],
            [0.00075, 0.0, 0.0015],
            [0.0, 0.0, 0.0],
        ]
    )

    stationary, xy_spread_m, z_spread_m, xy_limit_m, z_limit_m = (
        ArmMotionExecutor._vertical_feedback_stationarity(sample_xyz)
    )

    assert stationary is True
    assert xy_spread_m == pytest.approx(0.00075)
    assert z_spread_m == pytest.approx(0.0015)
    assert xy_limit_m == pytest.approx(0.00075)
    assert z_limit_m == pytest.approx(0.0015)


def test_vertical_feedback_stationarity_uses_maximum_pairwise_xy_distance(
    monkeypatch,
):
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_XY_TOLERANCE_M", 0.0015)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    sample_xyz = np.array(
        [
            [0.0, 0.0, 0.0],
            [0.0006, 0.0006, 0.0],
            [0.0, 0.0, 0.0],
        ]
    )

    stationary, xy_spread_m, _, _, _ = (
        ArmMotionExecutor._vertical_feedback_stationarity(sample_xyz)
    )

    assert stationary is False
    assert xy_spread_m == pytest.approx(np.hypot(0.0006, 0.0006))


def test_vertical_pregrasp_waits_for_recorded_drift_before_freezing(monkeypatch):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    target = np.array([0.0, 0.0, 0.100, 0.0, 0.0, 0.0])
    biased = target.copy()
    biased[:3] += [0.0, 0.007, -0.016]
    drifting_early = target.copy()
    drifting_early[:3] += [0.0, 0.0010, 0.0017]
    drifting_latest = target.copy()
    drifting_latest[:3] += [0.0, 0.0017, 0.0037]
    observed_poses = iter(
        [biased.copy() for _ in range(3)]
        + [drifting_early.copy(), drifting_early.copy(), drifting_latest.copy()]
        + [target.copy() for _ in range(3)]
    )
    executor.get_current_ee_pose_6d = lambda: next(observed_poses)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_XY_TOLERANCE_M", 0.0015)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_COMMAND_PERIOD_SEC", 0.05)
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_SETTLE_TIMEOUT_SEC", 1.5)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_MAX_OFFSET_M", 0.03)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_MAX_ITERATIONS", 3)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_SETTLE_SEC", 0.4)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_SAMPLE_COUNT", 3)
    monkeypatch.setattr("my_course_pkg.grasp.executor.time.sleep", lambda _: None)

    calibration = executor.converge_at_vertical_pregrasp(target)

    np.testing.assert_allclose(calibration.observed_median_xyz, target[:3])
    np.testing.assert_allclose(
        calibration.command_offset_xyz,
        [0.0, -0.007, 0.016],
    )
    assert calibration.command_count == 1
    assert len(executor.arm_api2_client.pose_commands) == 1


def test_vertical_pregrasp_requires_latest_sample_to_pass(monkeypatch):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    target = np.array([0.0, 0.0, 0.100, 0.0, 0.0, 0.0])
    median_pass = target.copy()
    median_pass[0] += 0.0014
    latest_fail = target.copy()
    latest_fail[0] += 0.0016
    observed_poses = iter(
        [median_pass.copy(), median_pass.copy(), latest_fail.copy()]
        + [target.copy() for _ in range(3)]
    )
    executor.get_current_ee_pose_6d = lambda: next(observed_poses)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_XY_TOLERANCE_M", 0.0015)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_COMMAND_PERIOD_SEC", 0.05)
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_SETTLE_TIMEOUT_SEC", 1.5)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_MAX_OFFSET_M", 0.03)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_MAX_ITERATIONS", 3)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_SETTLE_SEC", 0.4)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_SAMPLE_COUNT", 3)
    monkeypatch.setattr("my_course_pkg.grasp.executor.time.sleep", lambda _: None)

    calibration = executor.converge_at_vertical_pregrasp(target)

    np.testing.assert_allclose(calibration.command_offset_xyz, [-0.0014, 0.0, 0.0])
    assert calibration.command_count == 1
    assert len(executor.arm_api2_client.pose_commands) == 1


def test_vertical_pregrasp_stationarity_timeout_holds_without_correction(
    monkeypatch,
):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    target = np.array([0.0, 0.0, 0.100, 0.0, 0.0, 0.0])
    moving = target.copy()
    moving[0] += 0.002
    observed_poses = iter(
        [target.copy(), moving.copy(), target.copy()]
        + [target.copy(), moving.copy(), target.copy()]
    )
    executor.get_current_ee_pose_6d = lambda: next(observed_poses)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_XY_TOLERANCE_M", 0.0015)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_COMMAND_PERIOD_SEC", 0.05)
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_SETTLE_TIMEOUT_SEC", 1.5)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_MAX_OFFSET_M", 0.03)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_MAX_ITERATIONS", 3)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_SETTLE_SEC", 0.4)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_SAMPLE_COUNT", 3)
    monkeypatch.setattr("my_course_pkg.grasp.executor.time.sleep", lambda _: None)
    monotonic_times = iter([0.0, 1.5])
    monkeypatch.setattr(
        "my_course_pkg.grasp.executor.time.monotonic",
        lambda: next(monotonic_times),
    )

    with pytest.raises(VerticalApproachConvergenceError, match="stationary") as exc:
        executor.converge_at_vertical_pregrasp(target)

    assert exc.value.command_count == 0
    assert len(executor.arm_api2_client.pose_commands) == 1
    hold = executor.arm_api2_client.pose_commands[0]
    np.testing.assert_allclose(
        [hold.pose.position.x, hold.pose.position.y, hold.pose.position.z],
        target[:3],
    )


def test_vertical_pregrasp_compensation_resets_stationarity_deadline(monkeypatch):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    target = np.array([0.0, 0.0, 0.100, 0.0, 0.0, 0.0])
    moving = target.copy()
    moving[0] += 0.002
    biased = target.copy()
    biased[:3] += [0.0, 0.007, -0.016]
    moving_window = [target.copy(), moving.copy(), target.copy()]
    observed_poses = iter(
        moving_window
        + [biased.copy() for _ in range(3)]
        + moving_window
        + moving_window
        + [target.copy() for _ in range(3)]
    )
    executor.get_current_ee_pose_6d = lambda: next(observed_poses)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_XY_TOLERANCE_M", 0.0015)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_COMMAND_PERIOD_SEC", 0.05)
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_SETTLE_TIMEOUT_SEC", 1.5)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_MAX_OFFSET_M", 0.03)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_MAX_ITERATIONS", 3)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_SETTLE_SEC", 0.4)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_SAMPLE_COUNT", 3)
    monkeypatch.setattr("my_course_pkg.grasp.executor.time.sleep", lambda _: None)
    monotonic_times = iter([0.0, 1.0, 2.0])
    monkeypatch.setattr(
        "my_course_pkg.grasp.executor.time.monotonic",
        lambda: next(monotonic_times),
    )

    calibration = executor.converge_at_vertical_pregrasp(target)

    np.testing.assert_allclose(
        calibration.command_offset_xyz,
        [0.0, -0.007, 0.016],
    )
    assert calibration.command_count == 1
    assert len(executor.arm_api2_client.pose_commands) == 1


def test_vertical_pregrasp_accumulates_residual_corrections(monkeypatch):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    target = np.array([0.0, 0.0, 0.100, 0.0, 0.0, 0.0])
    first = target.copy()
    first[:3] += [0.004, 0.0, -0.010]
    second = target.copy()
    second[:3] += [0.002, 0.0, -0.004]
    observed_poses = iter(
        [first.copy() for _ in range(3)]
        + [second.copy() for _ in range(3)]
        + [target.copy() for _ in range(3)]
    )
    executor.get_current_ee_pose_6d = lambda: next(observed_poses)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_XY_TOLERANCE_M", 0.0015)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_COMMAND_PERIOD_SEC", 0.05)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_MAX_OFFSET_M", 0.03)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_MAX_ITERATIONS", 3)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_SETTLE_SEC", 0.4)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_SAMPLE_COUNT", 3)
    monkeypatch.setattr("my_course_pkg.grasp.executor.time.sleep", lambda _: None)

    calibration = executor.converge_at_vertical_pregrasp(target)

    np.testing.assert_allclose(
        calibration.command_offset_xyz,
        [-0.006, 0.0, 0.014],
    )
    assert calibration.command_count == 2


def test_vertical_pregrasp_uses_component_median_for_outlier(monkeypatch):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    target = np.array([0.0, 0.0, 0.100, 0.0, 0.0, 0.0])
    biased = target.copy()
    biased[:3] += [0.0, 0.007, -0.016]
    outlier = target.copy()
    outlier[:3] += [0.2, -0.2, 0.2]
    observed_poses = iter(
        [biased.copy(), outlier, biased.copy()]
        + [biased.copy() for _ in range(3)]
        + [target.copy() for _ in range(3)]
    )
    executor.get_current_ee_pose_6d = lambda: next(observed_poses)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_XY_TOLERANCE_M", 0.0015)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_COMMAND_PERIOD_SEC", 0.05)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_MAX_OFFSET_M", 0.03)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_MAX_ITERATIONS", 3)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_SETTLE_SEC", 0.4)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_SAMPLE_COUNT", 3)
    monkeypatch.setattr("my_course_pkg.grasp.executor.time.sleep", lambda _: None)

    calibration = executor.converge_at_vertical_pregrasp(target)

    np.testing.assert_allclose(
        calibration.command_offset_xyz,
        [0.0, -0.007, 0.016],
    )


def test_vertical_pregrasp_rejects_offset_above_cap_before_compensated_command(
    monkeypatch,
):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    target = np.zeros(6)
    actual = np.array([0.031, 0.0, 0.0, 0.0, 0.0, 0.0])
    executor.get_current_ee_pose_6d = lambda: actual.copy()
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_XY_TOLERANCE_M", 0.0015)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_COMMAND_PERIOD_SEC", 0.05)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_MAX_OFFSET_M", 0.03)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_MAX_ITERATIONS", 3)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_SETTLE_SEC", 0.4)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_SAMPLE_COUNT", 3)
    monkeypatch.setattr("my_course_pkg.grasp.executor.time.sleep", lambda _: None)

    with pytest.raises(VerticalApproachConvergenceError, match="maximum offset"):
        executor.converge_at_vertical_pregrasp(target)

    assert len(executor.arm_api2_client.pose_commands) == 1
    hold = executor.arm_api2_client.pose_commands[0]
    np.testing.assert_allclose(
        [hold.pose.position.x, hold.pose.position.y, hold.pose.position.z],
        actual[:3],
    )


def test_vertical_pregrasp_rejects_diverging_residual(monkeypatch):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    target = np.zeros(6)
    first = np.array([0.0, 0.0, -0.010, 0.0, 0.0, 0.0])
    diverged = np.array([0.0, 0.0, -0.014, 0.0, 0.0, 0.0])
    observed_poses = iter(
        [first.copy() for _ in range(3)]
        + [diverged.copy() for _ in range(3)]
    )
    executor.get_current_ee_pose_6d = lambda: next(observed_poses)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_XY_TOLERANCE_M", 0.0015)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_COMMAND_PERIOD_SEC", 0.05)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_MAX_OFFSET_M", 0.03)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_MAX_ITERATIONS", 3)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_SETTLE_SEC", 0.4)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_SAMPLE_COUNT", 3)
    monkeypatch.setattr("my_course_pkg.grasp.executor.time.sleep", lambda _: None)

    with pytest.raises(VerticalApproachConvergenceError, match="diverged"):
        executor.converge_at_vertical_pregrasp(target)

    assert len(executor.arm_api2_client.pose_commands) == 2


def test_vertical_pregrasp_iteration_exhaustion_holds_before_descent(monkeypatch):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    target = np.zeros(6)
    actual = np.array([0.0, 0.0, -0.006, 0.0, 0.0, 0.0])
    executor.get_current_ee_pose_6d = lambda: actual.copy()
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_XY_TOLERANCE_M", 0.0015)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_COMMAND_PERIOD_SEC", 0.05)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_MAX_OFFSET_M", 0.03)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_MAX_ITERATIONS", 3)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_SETTLE_SEC", 0.4)
    monkeypatch.setattr(executor_module, "VERTICAL_REANCHOR_SAMPLE_COUNT", 3)
    monkeypatch.setattr("my_course_pkg.grasp.executor.time.sleep", lambda _: None)

    with pytest.raises(VerticalApproachConvergenceError, match="iterations"):
        executor.converge_at_vertical_pregrasp(target)

    assert len(executor.arm_api2_client.pose_commands) == 4


def test_vertical_approach_uses_feedback_gated_five_mm_waypoints(monkeypatch):
    executor = object.__new__(ArmMotionExecutor)
    executor.enter_servo_pos_mode = lambda: pytest.fail(
        "vertical descent must reuse the calibrated servo mode"
    )
    observed = []
    executor._execute_vertical_approach_waypoint = (
        lambda pose, waypoint_index, waypoint_count, command_offset_xyz,
        final_z_bounds=None: observed.append(
            (
                np.asarray(pose, dtype=float),
                waypoint_index,
                waypoint_count,
                np.asarray(command_offset_xyz, dtype=float),
                final_z_bounds,
            )
        )
    )
    monkeypatch.setattr(
        executor_module,
        "VERTICAL_APPROACH_WAYPOINT_MAX_DIST_M",
        0.005,
    )
    start = np.array([0.0, 0.0, 0.100, 0.0, 0.0, 0.0])
    end = np.array([0.0, 0.0, 0.000, 0.0, 0.0, 0.0])

    command_offset_xyz = np.array([0.001, -0.002, 0.003])
    final_z_bounds = object()
    executor.move_vertical_approach(
        start,
        end,
        command_offset_xyz,
        final_z_bounds=final_z_bounds,
    )

    assert len(observed) == 20
    assert [item[1] for item in observed] == list(range(1, 21))
    assert all(item[2] == 20 for item in observed)
    assert all(np.array_equal(item[3], command_offset_xyz) for item in observed)
    assert all(item[4] is None for item in observed[:-1])
    assert observed[-1][4] is final_z_bounds
    commanded = np.vstack([item[0][:3] for item in observed])
    path = np.vstack([start[:3], commanded])
    assert np.max(np.linalg.norm(np.diff(path, axis=0), axis=1)) <= 0.005 + 1e-12
    np.testing.assert_allclose(commanded[-1], end[:3])


def test_vertical_waypoint_precommand_lateral_transient_waits_before_lowering(
    monkeypatch,
):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    observed_poses = [
        np.array([0.0029, 0.0, 0.100, 0.0, 0.0, 0.0]),
        np.array([0.0010, 0.0, 0.100, 0.0, 0.0, 0.0]),
        np.array([0.0010, 0.0, 0.097, 0.0, 0.0, 0.0]),
    ]
    observation_index = 0

    def get_current_pose():
        nonlocal observation_index
        expected_command_count = 0 if observation_index < 2 else 1
        assert len(executor.arm_api2_client.pose_commands) == expected_command_count
        pose = observed_poses[observation_index]
        observation_index += 1
        return pose.copy()

    target = np.array([0.0, 0.0, 0.095, 0.0, 0.0, 0.0])
    executor.get_current_ee_pose_6d = get_current_pose
    monkeypatch.setattr(executor_module, "VERTICAL_DESCENT_XY_TOLERANCE_M", 0.0020)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_SETTLE_TIMEOUT_SEC", 1.5)
    monkeypatch.setattr("my_course_pkg.grasp.executor.time.sleep", lambda _: None)

    actual = executor._execute_vertical_approach_waypoint(
        target,
        7,
        20,
        np.zeros(3),
    )

    assert len(executor.arm_api2_client.pose_commands) == 1
    lower_target = executor.arm_api2_client.pose_commands[0]
    np.testing.assert_allclose(
        [
            lower_target.pose.position.x,
            lower_target.pose.position.y,
            lower_target.pose.position.z,
        ],
        target[:3],
    )
    np.testing.assert_allclose(actual[:3], [0.0010, 0.0, 0.097])


def test_vertical_waypoint_precommand_persistent_lateral_error_times_out_without_lowering(
    monkeypatch,
):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    observed_poses = iter(
        [
            np.array([0.0029, 0.0, 0.100, 0.0, 0.0, 0.0]),
            np.array([0.0028, 0.0, 0.100, 0.0, 0.0, 0.0]),
        ]
    )
    target = np.array([0.0, 0.0, 0.095, 0.0, 0.0, 0.0])
    executor.get_current_ee_pose_6d = lambda: next(observed_poses)
    monkeypatch.setattr(executor_module, "VERTICAL_DESCENT_XY_TOLERANCE_M", 0.0020)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_SETTLE_TIMEOUT_SEC", 1.5)
    monkeypatch.setattr("my_course_pkg.grasp.executor.time.sleep", lambda _: None)
    monotonic_times = iter([0.0, 0.0, 1.5])
    monkeypatch.setattr(
        "my_course_pkg.grasp.executor.time.monotonic",
        lambda: next(monotonic_times),
    )

    with pytest.raises(
        VerticalApproachConvergenceError,
        match="timeout waiting for lateral convergence before lower waypoint",
    ) as exc_info:
        executor._execute_vertical_approach_waypoint(target, 7, 20, np.zeros(3))

    assert exc_info.value.xy_tolerance_m == pytest.approx(0.0020)
    assert len(executor.arm_api2_client.pose_commands) == 1
    hold = executor.arm_api2_client.pose_commands[0]
    np.testing.assert_allclose(
        [hold.pose.position.x, hold.pose.position.y, hold.pose.position.z],
        [0.0028, 0.0, 0.100],
    )


def test_vertical_waypoint_lateral_transient_waits_then_converges(monkeypatch):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    observed_poses = iter(
        [
            np.array([0.0, 0.0, 0.100, 0.0, 0.0, 0.0]),
            np.array([0.0021, 0.0, 0.0976, 0.0, 0.0, 0.0]),
            np.array([0.0010, 0.0, 0.0970, 0.0, 0.0, 0.0]),
        ]
    )
    executor.get_current_ee_pose_6d = lambda: next(observed_poses)
    target = np.array([0.0, 0.0, 0.095, 0.0, 0.0, 0.0])
    monkeypatch.setattr(executor_module, "VERTICAL_DESCENT_XY_TOLERANCE_M", 0.0015)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_SETTLE_TIMEOUT_SEC", 1.5)
    monkeypatch.setattr("my_course_pkg.grasp.executor.time.sleep", lambda _: None)

    actual = executor._execute_vertical_approach_waypoint(
        target,
        1,
        20,
        np.zeros(3),
    )

    assert len(executor.arm_api2_client.pose_commands) == 1
    lower_target = executor.arm_api2_client.pose_commands[0]
    np.testing.assert_allclose(
        [
            lower_target.pose.position.x,
            lower_target.pose.position.y,
            lower_target.pose.position.z,
        ],
        target[:3],
    )
    np.testing.assert_allclose(actual[:3], [0.0010, 0.0, 0.0970])


def test_vertical_waypoint_persistent_lateral_error_times_out_and_holds(monkeypatch):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    observed_poses = iter(
        [
            np.array([0.0, 0.0, 0.100, 0.0, 0.0, 0.0]),
            np.array([0.0021, 0.0, 0.0970, 0.0, 0.0, 0.0]),
            np.array([0.0021, 0.0, 0.0970, 0.0, 0.0, 0.0]),
        ]
    )
    executor.get_current_ee_pose_6d = lambda: next(observed_poses)
    target = np.array([0.0, 0.0, 0.095, 0.0, 0.0, 0.0])
    monkeypatch.setattr(executor_module, "VERTICAL_DESCENT_XY_TOLERANCE_M", 0.0020)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_SETTLE_TIMEOUT_SEC", 1.5)
    monkeypatch.setattr("my_course_pkg.grasp.executor.time.sleep", lambda _: None)
    monotonic_times = iter([0.0, 0.0, 1.5])
    monkeypatch.setattr(
        "my_course_pkg.grasp.executor.time.monotonic",
        lambda: next(monotonic_times),
    )

    with pytest.raises(
        VerticalApproachConvergenceError,
        match="timeout",
    ) as exc_info:
        executor._execute_vertical_approach_waypoint(target, 1, 20, np.zeros(3))

    assert exc_info.value.xy_tolerance_m == pytest.approx(0.0020)
    assert len(executor.arm_api2_client.pose_commands) == 2
    hold = executor.arm_api2_client.pose_commands[-1]
    np.testing.assert_allclose(
        [hold.pose.position.x, hold.pose.position.y, hold.pose.position.z],
        [0.0021, 0.0, 0.0970],
    )


def test_vertical_waypoint_xy_and_z_must_pass_in_same_sample(monkeypatch):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    observed_poses = iter(
        [
            np.array([0.0, 0.0, 0.100, 0.0, 0.0, 0.0]),
            np.array([0.0010, 0.0, 0.100, 0.0, 0.0, 0.0]),
            np.array([0.0021, 0.0, 0.0970, 0.0, 0.0, 0.0]),
            np.array([0.0010, 0.0, 0.0970, 0.0, 0.0, 0.0]),
        ]
    )
    executor.get_current_ee_pose_6d = lambda: next(observed_poses)
    target = np.array([0.0, 0.0, 0.095, 0.0, 0.0, 0.0])
    monkeypatch.setattr(executor_module, "VERTICAL_DESCENT_XY_TOLERANCE_M", 0.0020)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_SETTLE_TIMEOUT_SEC", 1.5)
    monkeypatch.setattr("my_course_pkg.grasp.executor.time.sleep", lambda _: None)
    monotonic_times = iter([0.0, 0.1, 0.2])
    monkeypatch.setattr(
        "my_course_pkg.grasp.executor.time.monotonic",
        lambda: next(monotonic_times),
    )

    actual = executor._execute_vertical_approach_waypoint(
        target,
        1,
        20,
        np.zeros(3),
    )

    assert len(executor.arm_api2_client.pose_commands) == 1
    np.testing.assert_allclose(actual[:3], [0.0010, 0.0, 0.0970])


def test_vertical_waypoint_z_timeout_holds_and_aborts(monkeypatch):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    actual = np.array([0.0, 0.0, 0.100, 0.0, 0.0, 0.0])
    target = np.array([0.0, 0.0, 0.095, 0.0, 0.0, 0.0])
    executor.get_current_ee_pose_6d = lambda: actual.copy()
    monkeypatch.setattr(executor_module, "VERTICAL_DESCENT_XY_TOLERANCE_M", 0.0020)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_SETTLE_TIMEOUT_SEC", 1.5)
    monotonic_times = iter([0.0, 1.5])
    monkeypatch.setattr(
        "my_course_pkg.grasp.executor.time.monotonic",
        lambda: next(monotonic_times),
    )

    with pytest.raises(VerticalApproachConvergenceError, match="timeout"):
        executor._execute_vertical_approach_waypoint(target, 1, 20, np.zeros(3))

    assert len(executor.arm_api2_client.pose_commands) == 2
    hold = executor.arm_api2_client.pose_commands[-1]
    np.testing.assert_allclose(
        [hold.pose.position.x, hold.pose.position.y, hold.pose.position.z],
        actual[:3],
    )


def test_vertical_waypoint_offsets_command_but_gates_against_nominal(monkeypatch):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    target = np.array([0.0, 0.0, 0.095, 0.1, -0.2, 0.3])
    command_offset_xyz = np.array([0.001, -0.002, 0.003])
    observed_poses = iter(
        [
            np.array([0.0, 0.0, 0.100, 0.1, -0.2, 0.3]),
            target.copy(),
        ]
    )
    executor.get_current_ee_pose_6d = lambda: next(observed_poses)
    monkeypatch.setattr(executor_module, "VERTICAL_DESCENT_XY_TOLERANCE_M", 0.0015)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_SETTLE_TIMEOUT_SEC", 1.5)

    executor._execute_vertical_approach_waypoint(
        target,
        1,
        20,
        command_offset_xyz,
    )

    assert len(executor.arm_api2_client.pose_commands) == 1
    command = executor.arm_api2_client.pose_commands[0]
    np.testing.assert_allclose(
        [command.pose.position.x, command.pose.position.y, command.pose.position.z],
        target[:3] + command_offset_xyz,
    )
    expected = executor_module._pose6d_to_posestamped_msg(target, frame_id="world")
    np.testing.assert_allclose(
        [
            command.pose.orientation.x,
            command.pose.orientation.y,
            command.pose.orientation.z,
            command.pose.orientation.w,
        ],
        [
            expected.pose.orientation.x,
            expected.pose.orientation.y,
            expected.pose.orientation.z,
            expected.pose.orientation.w,
        ],
    )


def test_vertical_final_z_bounds_are_derived_from_live_plan_geometry():
    bounds = ArmMotionExecutor._vertical_final_z_bounds(
        vertical_plan_with_bounds(),
    )

    assert bounds.valid
    assert bounds.reason == "valid"
    assert bounds.bottom_z == pytest.approx(0.8900)
    assert bounds.center_z == pytest.approx(0.9053)
    assert bounds.top_z == pytest.approx(0.9206)


@pytest.mark.parametrize("signed_z_error_m", [-0.0029, 0.0029])
def test_vertical_final_z_ordinary_gate_does_not_require_bounds(
    monkeypatch,
    signed_z_error_m,
):
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    target = np.array([0.0, 0.0, 0.8980, 0.0, 0.0, 0.0])
    actual = target.copy()
    actual[2] += signed_z_error_m

    decision = ArmMotionExecutor._vertical_final_z_decision(
        target,
        actual,
        None,
    )

    assert decision.passed
    assert decision.mode == "ordinary"
    assert decision.signed_z_error_m == pytest.approx(signed_z_error_m)
    assert decision.negative_tolerance_m == pytest.approx(0.003)
    assert decision.positive_tolerance_m == pytest.approx(0.003)


def test_vertical_final_z_accepts_gelatin_high_residual_inside_live_bounds(
    monkeypatch,
):
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    target = np.array([0.0, 0.0, 0.8980, 0.0, 0.0, 0.0])
    actual = np.array([0.0006, 0.0, 0.9032, 0.0, 0.0, 0.0])
    bounds = ArmMotionExecutor._vertical_final_z_bounds(
        vertical_plan_with_bounds(),
    )

    decision = ArmMotionExecutor._vertical_final_z_decision(
        target,
        actual,
        bounds,
    )

    assert decision.passed
    assert decision.mode == "controlled_center"
    assert decision.signed_z_error_m == pytest.approx(0.0052)
    assert decision.negative_tolerance_m == pytest.approx(0.003)
    assert decision.positive_tolerance_m == pytest.approx(0.0073)
    assert decision.center_positive_limit_m == pytest.approx(0.0073)
    assert decision.reason == "controlled positive residual at or below live center"


def test_vertical_final_z_accepts_actual_exactly_at_live_center(monkeypatch):
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    target = np.array([0.0, 0.0, 0.8980, 0.0, 0.0, 0.0])
    actual = np.array([0.0, 0.0, 0.9053, 0.0, 0.0, 0.0])
    bounds = ArmMotionExecutor._vertical_final_z_bounds(
        vertical_plan_with_bounds(),
    )

    decision = ArmMotionExecutor._vertical_final_z_decision(
        target,
        actual,
        bounds,
    )

    assert decision.passed
    assert decision.mode == "controlled_center"
    assert decision.signed_z_error_m == pytest.approx(0.0073)
    assert decision.center_positive_limit_m == pytest.approx(0.0073)


def test_vertical_final_z_rejects_above_center_even_within_retired_five_mm_cap(
    monkeypatch,
):
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    target = np.array([0.0, 0.0, 0.8980, 0.0, 0.0, 0.0])
    actual = np.array([0.0, 0.0, 0.9020, 0.0, 0.0, 0.0])
    bounds = ArmMotionExecutor._vertical_final_z_bounds(
        vertical_plan_with_bounds(bottom_z=0.8900, center_z=0.9015),
    )

    decision = ArmMotionExecutor._vertical_final_z_decision(
        target,
        actual,
        bounds,
    )

    assert not decision.passed
    assert decision.signed_z_error_m == pytest.approx(0.0040)
    assert decision.center_positive_limit_m == pytest.approx(0.0035)
    assert decision.reason == "actual Z is above live center"


@pytest.mark.parametrize(
    ("actual_z", "expected_reason"),
    [
        (0.8934, "downward residual exceeds ordinary tolerance"),
        (0.9054, "actual Z is above live center"),
    ],
)
def test_vertical_final_z_rejects_out_of_policy_residuals(
    monkeypatch,
    actual_z,
    expected_reason,
):
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    target = np.array([0.0, 0.0, 0.8980, 0.0, 0.0, 0.0])
    actual = np.array([0.0, 0.0, actual_z, 0.0, 0.0, 0.0])
    bounds = ArmMotionExecutor._vertical_final_z_bounds(
        vertical_plan_with_bounds(),
    )

    decision = ArmMotionExecutor._vertical_final_z_decision(
        target,
        actual,
        bounds,
    )

    assert not decision.passed
    assert decision.mode == "rejected"
    assert decision.reason == expected_reason


@pytest.mark.parametrize(
    ("plan", "expected_reason"),
    [
        (
            SimpleNamespace(debug_info={"grasp_profile": "vertical"}),
            "missing live vertical bounds",
        ),
        (
            vertical_plan_with_bounds(bottom_z=0.9053, center_z=0.9053),
            "live vertical bounds are not strictly ordered",
        ),
        (
            vertical_plan_with_bounds(bottom_z=float("nan")),
            "live vertical bounds are non-finite",
        ),
        (
            SimpleNamespace(debug_info={"grasp_profile": "side"}),
            "non-vertical grasp profile",
        ),
    ],
)
def test_vertical_final_z_rejects_high_residual_without_valid_bounds(
    monkeypatch,
    plan,
    expected_reason,
):
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    target = np.array([0.0, 0.0, 0.8980, 0.0, 0.0, 0.0])
    actual = np.array([0.0, 0.0, 0.9026, 0.0, 0.0, 0.0])
    bounds = ArmMotionExecutor._vertical_final_z_bounds(plan)

    decision = ArmMotionExecutor._vertical_final_z_decision(
        target,
        actual,
        bounds,
    )

    assert not decision.passed
    assert decision.reason == expected_reason


@pytest.mark.parametrize(
    ("bottom_z", "center_z", "target_z", "actual_z", "expected_reason"),
    [
        (0.8900, 0.9000, 0.8850, 0.8896, "target Z is outside live bounds"),
        (0.8900, 0.8990, 0.9040, 0.9086, "actual Z is above live center"),
    ],
)
def test_vertical_final_z_rejects_target_or_actual_outside_bounds(
    monkeypatch,
    bottom_z,
    center_z,
    target_z,
    actual_z,
    expected_reason,
):
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    target = np.array([0.0, 0.0, target_z, 0.0, 0.0, 0.0])
    actual = np.array([0.0, 0.0, actual_z, 0.0, 0.0, 0.0])
    bounds = ArmMotionExecutor._vertical_final_z_bounds(
        vertical_plan_with_bounds(bottom_z=bottom_z, center_z=center_z),
    )

    decision = ArmMotionExecutor._vertical_final_z_decision(
        target,
        actual,
        bounds,
    )

    assert not decision.passed
    assert decision.reason == expected_reason


def test_vertical_final_waypoint_accepts_settled_high_residual_without_hold(
    monkeypatch,
):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    target = np.array([0.0, 0.0, 0.8980, 0.0, 0.0, 0.0])
    actual = np.array([0.0006, 0.0, 0.9032, 0.0, 0.0, 0.0])
    executor.get_current_ee_pose_6d = lambda: actual.copy()
    bounds = ArmMotionExecutor._vertical_final_z_bounds(
        vertical_plan_with_bounds(),
    )
    monkeypatch.setattr(executor_module, "VERTICAL_DESCENT_XY_TOLERANCE_M", 0.002)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_SETTLE_TIMEOUT_SEC", 1.5)
    monotonic_times = iter([0.0, 1.5])
    monkeypatch.setattr(
        "my_course_pkg.grasp.executor.time.monotonic",
        lambda: next(monotonic_times),
    )

    verified = executor._execute_vertical_approach_waypoint(
        target,
        21,
        21,
        np.zeros(3),
        final_z_bounds=bounds,
    )

    np.testing.assert_allclose(verified, actual)
    assert len(executor.arm_api2_client.pose_commands) == 1
    assert any(
        "z_acceptance_mode=controlled_center" in message
        and "no_additional_lower_command=true" in message
        for message in executor.node.logger.infos
    )


def test_vertical_nonfinal_waypoint_keeps_three_mm_z_gate(monkeypatch):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    target = np.array([0.0, 0.0, 0.8980, 0.0, 0.0, 0.0])
    actual = np.array([0.0013, 0.0, 0.9026, 0.0, 0.0, 0.0])
    executor.get_current_ee_pose_6d = lambda: actual.copy()
    bounds = ArmMotionExecutor._vertical_final_z_bounds(
        vertical_plan_with_bounds(),
    )
    monkeypatch.setattr(executor_module, "VERTICAL_DESCENT_XY_TOLERANCE_M", 0.002)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_SETTLE_TIMEOUT_SEC", 1.5)
    monotonic_times = iter([0.0, 1.5])
    monkeypatch.setattr(
        "my_course_pkg.grasp.executor.time.monotonic",
        lambda: next(monotonic_times),
    )

    with pytest.raises(VerticalApproachConvergenceError) as exc_info:
        executor._execute_vertical_approach_waypoint(
            target,
            20,
            21,
            np.zeros(3),
            final_z_bounds=bounds,
        )

    assert exc_info.value.z_positive_tolerance_m == pytest.approx(0.003)
    assert len(executor.arm_api2_client.pose_commands) == 2


def test_vertical_descent_accepts_controlled_xy_error_between_gate_limits(
    monkeypatch,
):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    target = np.array([0.0, 0.0, 0.095, 0.0, 0.0, 0.0])
    observed_poses = iter(
        [
            np.array([0.0016, 0.0, 0.100, 0.0, 0.0, 0.0]),
            np.array([0.0016, 0.0, 0.095, 0.0, 0.0, 0.0]),
        ]
    )
    executor.get_current_ee_pose_6d = lambda: next(observed_poses)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_XY_TOLERANCE_M", 0.0015)
    monkeypatch.setattr(executor_module, "VERTICAL_DESCENT_XY_TOLERANCE_M", 0.0020)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_SETTLE_TIMEOUT_SEC", 1.5)

    actual = executor._execute_vertical_approach_waypoint(
        target,
        18,
        20,
        np.zeros(3),
    )

    np.testing.assert_allclose(actual[:3], [0.0016, 0.0, 0.095])
    assert len(executor.arm_api2_client.pose_commands) == 1
    assert any(
        "xy_tolerance_m=0.0020" in message
        for message in executor.node.logger.infos
    )


def test_vertical_final_verification_accepts_controlled_two_mm_xy_gate(
    monkeypatch,
):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    actual = np.array([0.0016, 0.0, 0.0029, 0.0, 0.0, 0.0])
    target = np.zeros(6)
    executor.get_current_ee_pose_6d = lambda: actual.copy()
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_XY_TOLERANCE_M", 0.0015)
    monkeypatch.setattr(executor_module, "VERTICAL_DESCENT_XY_TOLERANCE_M", 0.0020)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)

    verified = executor.verify_vertical_final_grasp(target)

    np.testing.assert_allclose(verified, actual)
    assert executor.arm_api2_client.pose_commands == []
    assert any(
        "xy_tolerance_m=0.0020" in message
        for message in executor.node.logger.infos
    )


def test_vertical_final_verification_accepts_bounded_high_z_residual(
    monkeypatch,
):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    actual = np.array([0.0006, 0.0, 0.9032, 0.0, 0.0, 0.0])
    target = np.array([0.0, 0.0, 0.8980, 0.0, 0.0, 0.0])
    executor.get_current_ee_pose_6d = lambda: actual.copy()
    bounds = ArmMotionExecutor._vertical_final_z_bounds(
        vertical_plan_with_bounds(),
    )
    monkeypatch.setattr(executor_module, "VERTICAL_DESCENT_XY_TOLERANCE_M", 0.0020)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)

    verified = executor.verify_vertical_final_grasp(
        target,
        final_z_bounds=bounds,
    )

    np.testing.assert_allclose(verified, actual)
    assert executor.arm_api2_client.pose_commands == []
    assert any(
        "signed_z_error_m=+0.0052" in message
        and "z_negative_tolerance_m=0.0030" in message
        and "z_positive_tolerance_m=0.0073" in message
        and "z_center_positive_limit_m=0.0073" in message
        and "z_acceptance_mode=controlled_center" in message
        and "bounds_bottom_z=0.8900" in message
        and "bounds_top_z=0.9206" in message
        for message in executor.node.logger.infos
    )


@pytest.mark.parametrize("actual_z", [0.8934, 0.9026])
def test_vertical_final_verification_rejects_unsafe_or_unbounded_residual(
    monkeypatch,
    actual_z,
):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    actual = np.array([0.0, 0.0, actual_z, 0.0, 0.0, 0.0])
    target = np.array([0.0, 0.0, 0.8980, 0.0, 0.0, 0.0])
    executor.get_current_ee_pose_6d = lambda: actual.copy()
    monkeypatch.setattr(executor_module, "VERTICAL_DESCENT_XY_TOLERANCE_M", 0.0020)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)

    with pytest.raises(VerticalApproachConvergenceError) as exc_info:
        executor.verify_vertical_final_grasp(target, final_z_bounds=None)

    assert exc_info.value.z_negative_tolerance_m == pytest.approx(0.003)
    assert exc_info.value.z_positive_tolerance_m == pytest.approx(0.003)
    assert exc_info.value.z_center_positive_limit_m is None
    assert len(executor.arm_api2_client.pose_commands) == 1


def test_vertical_final_verification_rejects_error_below_global_tolerance(
    monkeypatch,
):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    actual = np.array([0.0126, 0.0, 0.0, 0.0, 0.0, 0.0])
    target = np.zeros(6)
    executor.get_current_ee_pose_6d = lambda: actual.copy()
    monkeypatch.setattr(executor_module, "FINAL_APPROACH_POSITION_TOLERANCE_M", 0.018)
    monkeypatch.setattr(executor_module, "VERTICAL_DESCENT_XY_TOLERANCE_M", 0.0020)
    monkeypatch.setattr(executor_module, "VERTICAL_APPROACH_Z_TOLERANCE_M", 0.003)

    with pytest.raises(
        VerticalApproachConvergenceError,
        match="final_verify",
    ) as exc_info:
        executor.verify_vertical_final_grasp(target)

    assert exc_info.value.xy_tolerance_m == pytest.approx(0.0020)
    assert exc_info.value.z_tolerance_m == pytest.approx(0.003)
    assert exc_info.value.command_count == 0
    assert len(executor.arm_api2_client.pose_commands) == 1


def test_vertical_plan_orders_reanchor_before_approach_and_close(monkeypatch):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.debug_stop_reached = False
    events = []
    executor.execute_step = lambda step: events.append(("generic", step.name))
    executor.enter_servo_pos_mode = lambda: events.append(("mode", "cartesian"))
    command_offset_xyz = np.array([0.001, -0.002, 0.003])

    def converge_at_vertical_pregrasp(pose):
        events.append(("pregrasp", np.asarray(pose, dtype=float).copy()))
        return VerticalApproachCalibration(
            observed_median_xyz=np.asarray(pose, dtype=float)[:3].copy(),
            command_offset_xyz=command_offset_xyz.copy(),
            command_count=1,
        )

    executor.converge_at_vertical_pregrasp = converge_at_vertical_pregrasp
    executor.move_vertical_approach = (
        lambda start, end, offset, final_z_bounds=None: events.append(
        (
            "approach",
            np.asarray(end, dtype=float).copy(),
            np.asarray(offset, dtype=float).copy(),
            final_z_bounds,
        )
    )
    )
    executor.verify_vertical_final_grasp = (
        lambda pose, final_z_bounds=None: events.append(
            (
                "final_verify",
                np.asarray(pose, dtype=float).copy(),
                final_z_bounds,
            )
        )
    )
    for name in (
        "GRASP_DEBUG_STOP_AT_PREGRASP",
        "GRASP_DEBUG_STOP_AT_GRASP",
        "GRASP_DEBUG_STOP_AFTER_CLOSE",
        "GRASP_DEBUG_STOP_AFTER_LIFT",
        "GRASP_DEBUG_STOP_BEFORE_RELEASE",
    ):
        monkeypatch.setattr(executor_module, name, False, raising=False)
    pregrasp = np.array([0.0, 0.0, 0.1, 0.0, 0.0, 0.0])
    grasp = np.zeros(6)
    plan = SimpleNamespace(
        pre_grasp_pose_6d=pregrasp,
        grasp_pose_6d=grasp,
        debug_info={
            "grasp_profile": "vertical",
            "vertical_target_bounds_center": np.array([0.0, 0.0, 0.01]),
            "vertical_bounds_bottom_z": -0.01,
        },
        steps=[
            PlanStep(
                name="move_to_pre_grasp",
                action="move",
                mode="moveit",
                end_pose_6d=pregrasp,
            ),
            PlanStep(
                name="approach_grasp",
                action="move",
                mode="cartesian",
                start_pose_6d=pregrasp,
                end_pose_6d=grasp,
            ),
            PlanStep(
                name="close_gripper_at_grasp",
                action="gripper",
                gripper_position=0.79,
            ),
        ],
    )

    executor.execute_plan(plan)

    assert [event[0] for event in events] == [
        "generic",
        "mode",
        "pregrasp",
        "approach",
        "final_verify",
        "generic",
    ]
    np.testing.assert_allclose(events[3][2], command_offset_xyz)
    assert events[3][3].valid
    assert events[4][2] is events[3][3]


def test_non_vertical_plans_use_global_final_approach_tolerance(monkeypatch):
    executor = object.__new__(ArmMotionExecutor)
    monkeypatch.setattr(
        executor_module,
        "FINAL_APPROACH_POSITION_TOLERANCE_M",
        0.018,
    )

    centered_plan = SimpleNamespace(debug_info={"grasp_profile": "centered"})
    side_plan = SimpleNamespace(debug_info={"grasp_profile": "side"})
    legacy_plan = SimpleNamespace()

    assert executor._final_approach_position_tolerance_m(centered_plan) == pytest.approx(
        0.018
    )
    assert executor._final_approach_position_tolerance_m(side_plan) == pytest.approx(
        0.018
    )
    assert executor._final_approach_position_tolerance_m(legacy_plan) == pytest.approx(
        0.018
    )


def test_plan_passes_global_tolerance_to_convergence(monkeypatch):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    captured = {}

    def converge(target_pose_6d, position_tolerance_m=None):
        captured["target_pose_6d"] = target_pose_6d
        captured["position_tolerance_m"] = position_tolerance_m
        return target_pose_6d

    executor.converge_at_final_grasp = converge
    monkeypatch.setattr(
        executor_module,
        "FINAL_APPROACH_POSITION_TOLERANCE_M",
        0.018,
    )
    grasp_pose_6d = np.array([0.1, 0.2, 0.3, 0.0, 0.0, 0.0])
    plan = SimpleNamespace(
        grasp_pose_6d=grasp_pose_6d,
        debug_info={"grasp_profile": "centered"},
    )

    executor._converge_plan_at_final_grasp(plan)

    np.testing.assert_allclose(captured["target_pose_6d"], grasp_pose_6d)
    assert captured["position_tolerance_m"] == pytest.approx(0.018)
    assert any(
        "profile=centered" in message and "position_tolerance_m=0.0180" in message
        for message in executor.node.logger.infos
    )


def test_elevated_return_does_not_reuse_initial_grasp_convergence_gate():
    grasp_pose_6d = np.array([0.1, 0.2, 0.3, 0.0, 0.0, 0.0])
    release_pose_6d = grasp_pose_6d.copy()
    release_pose_6d[2] += 0.02
    plan = SimpleNamespace(grasp_pose_6d=grasp_pose_6d)
    approach_step = PlanStep(
        name="approach_grasp",
        action="move",
        mode="cartesian",
        end_pose_6d=grasp_pose_6d,
    )
    return_step = PlanStep(
        name="return_to_grasp",
        action="move",
        mode="cartesian",
        end_pose_6d=release_pose_6d,
    )

    assert ArmMotionExecutor._step_reaches_final_grasp(plan, approach_step)
    assert not ArmMotionExecutor._step_reaches_final_grasp(plan, return_step)


def test_final_approach_convergence_timeout_blocks_gripper_close(monkeypatch):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.arm_api2_client = DummyArmClient()
    executor.gripper_controller = DummyGripperController()
    executor.debug_stop_reached = False
    executed = []
    executor.execute_step = lambda step: executed.append(step.name)
    executor.get_current_ee_pose_6d = lambda: np.array(
        [0.02, 0.0, 0.0, 0.0, 0.0, 0.0]
    )
    monkeypatch.setattr(
        executor_module,
        "FINAL_APPROACH_POSITION_TOLERANCE_M",
        0.005,
    )
    monkeypatch.setattr(
        executor_module,
        "FINAL_APPROACH_SETTLE_TIMEOUT_SEC",
        1.5,
    )
    monkeypatch.setattr(
        executor_module,
        "FINAL_APPROACH_COMMAND_PERIOD_SEC",
        0.05,
    )
    monotonic_times = iter([0.0, 0.0, 1.5])
    monkeypatch.setattr(
        "my_course_pkg.grasp.executor.time.monotonic",
        lambda: next(monotonic_times),
    )
    monkeypatch.setattr("my_course_pkg.grasp.executor.time.sleep", lambda _: None)
    monkeypatch.setattr(
        executor_module,
        "GRASP_DEBUG_STOP_AT_GRASP",
        False,
        raising=False,
    )
    monkeypatch.setattr(
        executor_module,
        "GRASP_DEBUG_STOP_AFTER_CLOSE",
        False,
        raising=False,
    )
    grasp_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    plan = SimpleNamespace(
        grasp_pose_6d=grasp_pose,
        steps=[
            PlanStep(
                name="approach_grasp",
                action="move",
                mode="cartesian",
                end_pose_6d=grasp_pose.copy(),
            ),
            PlanStep(
                name="close_gripper_at_grasp",
                action="gripper",
                gripper_position=0.79,
            ),
        ],
    )

    with pytest.raises(FinalApproachConvergenceError):
        executor.execute_plan(plan)

    assert executed == ["approach_grasp"]
    assert len(executor.arm_api2_client.pose_commands) == 1


def test_passive_hold_waits_without_sending_a_servo_pose(monkeypatch):
    executor = object.__new__(ArmMotionExecutor)
    slept_for = []
    executor.hold_pose = lambda pose, duration: pytest.fail("servo hold was used")
    monkeypatch.setattr(
        "my_course_pkg.grasp.executor.time.sleep",
        lambda duration: slept_for.append(duration),
    )
    monkeypatch.setattr(
        "my_course_pkg.grasp.executor._pose6d_to_posestamped_msg",
        lambda pose, frame_id: object(),
    )

    executor.execute_step(
        PlanStep(
            name="hold_after_close",
            action="hold",
            mode="passive",
            pose_6d=[0.1, 0.2, 0.3, 0.0, 0.0, 0.0],
            duration_sec=0.25,
        )
    )

    assert slept_for == [0.25]


def test_debug_stop_at_grasp_skips_close_lift_and_remaining_steps(monkeypatch):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executed = []
    debug_stages = []
    executor.execute_step = lambda step: executed.append(step.name)
    executor._log_grasp_debug = lambda plan, stage: debug_stages.append(stage)
    executor.converge_at_final_grasp = lambda pose, **kwargs: pose
    executor.debug_stop_reached = False
    monkeypatch.setattr(
        executor_module,
        "GRASP_DEBUG_STOP_AT_GRASP",
        True,
        raising=False,
    )
    monkeypatch.setattr(
        executor_module,
        "GRASP_DEBUG_STOP_AFTER_CLOSE",
        False,
        raising=False,
    )
    grasp_pose = np.array([0.1, 0.2, 0.3, 0.0, 0.0, 0.0])
    plan = SimpleNamespace(
        grasp_pose_6d=grasp_pose,
        steps=[
            PlanStep(
                name="open_gripper_before_approach",
                action="gripper",
                gripper_position=0.0,
            ),
            PlanStep(
                name="move_to_pre_grasp",
                action="move",
                mode="moveit",
                end_pose_6d=grasp_pose.copy(),
            ),
            PlanStep(
                name="close_gripper_at_grasp",
                action="gripper",
                gripper_position=0.79,
            ),
            PlanStep(name="lift_after_grasp", action="move", mode="cartesian"),
        ],
    )

    executor.execute_plan(plan)

    assert executed == ["move_to_pre_grasp"]
    assert debug_stages == ["before_final_grasp", "after_final_grasp"]
    assert executor.debug_stop_reached is True


def test_debug_stop_after_close_skips_lift_and_remaining_steps(monkeypatch):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executed = []
    debug_stages = []
    executor.execute_step = lambda step: executed.append(step.name)
    executor._log_grasp_debug = lambda plan, stage: debug_stages.append(stage)
    executor.converge_at_final_grasp = lambda pose, **kwargs: pose
    executor.debug_stop_reached = False
    monkeypatch.setattr(
        executor_module,
        "GRASP_DEBUG_STOP_AT_GRASP",
        False,
        raising=False,
    )
    monkeypatch.setattr(
        executor_module,
        "GRASP_DEBUG_STOP_AFTER_CLOSE",
        True,
        raising=False,
    )
    grasp_pose = np.array([0.1, 0.2, 0.3, 0.0, 0.0, 0.0])
    plan = SimpleNamespace(
        grasp_pose_6d=grasp_pose,
        steps=[
            PlanStep(
                name="move_to_pre_grasp",
                action="move",
                mode="moveit",
                end_pose_6d=grasp_pose.copy(),
            ),
            PlanStep(
                name="close_gripper_at_grasp",
                action="gripper",
                gripper_position=0.79,
            ),
            PlanStep(
                name="hold_after_close",
                action="hold",
                mode="passive",
                duration_sec=0.01,
            ),
            PlanStep(name="lift_after_grasp", action="move", mode="cartesian"),
        ],
    )

    executor.execute_plan(plan)

    assert executed == [
        "move_to_pre_grasp",
        "close_gripper_at_grasp",
        "hold_after_close",
    ]
    assert debug_stages == ["before_final_grasp", "after_final_grasp", "after_close"]
    assert executor.debug_stop_reached is True


def test_debug_stop_at_pregrasp_skips_approach_and_gripper(monkeypatch):
    executor = object.__new__(ArmMotionExecutor)
    executed = []
    stages = []
    executor.execute_step = lambda step: executed.append(step.name)
    executor._log_grasp_debug = lambda plan, stage: stages.append(stage)
    executor.debug_stop_reached = False
    monkeypatch.setattr(executor_module, "GRASP_DEBUG_STOP_AT_PREGRASP", True, raising=False)
    monkeypatch.setattr(executor_module, "GRASP_DEBUG_STOP_AT_GRASP", False, raising=False)
    monkeypatch.setattr(executor_module, "GRASP_DEBUG_STOP_AFTER_CLOSE", False, raising=False)
    plan = SimpleNamespace(
        grasp_pose_6d=np.array([0.1, 0.2, 0.3, 0.0, 0.0, 0.0]),
        steps=[
            PlanStep(name="open_gripper_before_approach", action="gripper", gripper_position=0.0),
            PlanStep(name="move_to_pre_grasp", action="move", mode="moveit"),
            PlanStep(name="approach_grasp", action="move", mode="cartesian"),
            PlanStep(name="close_gripper_at_grasp", action="gripper", gripper_position=0.79),
        ],
    )

    executor.execute_plan(plan)

    assert executed == ["move_to_pre_grasp"]
    assert stages == ["at_pregrasp"]
    assert executor.debug_stop_reached is True


def test_close_gripper_accepts_structured_stalled_contact_result():
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.gripper_controller = SimpleNamespace(
        send_gripper_command=lambda position: gripper_result(
            accepted=True,
            target_position=position,
            actual_position=0.195,
            stalled=True,
            reached_goal=False,
            action_status=6,
        )
    )

    executor.execute_step(
        PlanStep(
            name="close_gripper_at_grasp",
            action="gripper",
            gripper_position=0.79,
        )
    )


@pytest.mark.parametrize("actual_position", [0.0, 0.038])
def test_pear_close_readiness_rejects_early_stall_before_hold_or_lift(
    monkeypatch,
    actual_position,
):
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.gripper_controller = SimpleNamespace(
        send_gripper_command=lambda position: gripper_result(
            accepted=True,
            target_position=position,
            actual_position=actual_position,
            stalled=True,
            reached_goal=False,
            action_status=6,
        )
    )
    executed_holds = []
    executor.hold_pose = lambda pose, duration: executed_holds.append(duration)
    monkeypatch.setattr(executor_module, "GRASP_DEBUG_STOP_AT_PREGRASP", False)
    monkeypatch.setattr(executor_module, "GRASP_DEBUG_STOP_AT_GRASP", False)
    monkeypatch.setattr(executor_module, "GRASP_DEBUG_STOP_AFTER_CLOSE", False)
    monkeypatch.setattr(executor_module, "GRASP_DEBUG_STOP_AFTER_LIFT", False)
    monkeypatch.setattr(executor_module, "GRASP_DEBUG_STOP_BEFORE_RELEASE", False)
    plan = SimpleNamespace(
        grasp_pose_6d=np.zeros(6),
        debug_info={
            "object_name": "pear",
            "grasp_profile": "round_top",
            "pear_projected_width_m": 0.066546,
            "pear_expected_close_position_rad": 0.172,
            "pear_minimum_close_position_rad": 0.162,
        },
        steps=[
            PlanStep(
                name="close_gripper_at_grasp",
                action="gripper",
                gripper_position=0.79,
            ),
            PlanStep(
                name="hold_after_close",
                action="hold",
                pose_6d=np.zeros(6),
                duration_sec=0.1,
            ),
        ],
    )

    with pytest.raises(GripperCommandError, match="bilateral-contact minimum"):
        executor.execute_plan(plan)

    assert executed_holds == []


def test_pear_close_readiness_accepts_contact_above_candidate_minimum():
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    step = PlanStep(
        name="close_gripper_at_grasp",
        action="gripper",
        gripper_position=0.79,
    )
    plan = SimpleNamespace(
        debug_info={
            "object_name": "pear",
            "pear_projected_width_m": 0.066546,
            "pear_expected_close_position_rad": 0.172,
            "pear_minimum_close_position_rad": 0.162,
        }
    )
    result = gripper_result(
        accepted=True,
        target_position=0.79,
        actual_position=0.170,
        stalled=True,
        reached_goal=False,
        action_status=6,
    )

    executor._validate_pear_close_result(plan, step, result)

    assert "Pear close readiness passed" in executor.node.logger.infos[-1]


def test_non_pear_close_does_not_use_pear_readiness_fields():
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    step = PlanStep(
        name="close_gripper_at_grasp",
        action="gripper",
        gripper_position=0.79,
    )
    plan = SimpleNamespace(debug_info={"object_name": "apple"})
    result = gripper_result(
        accepted=True,
        target_position=0.79,
        actual_position=0.038,
        stalled=True,
        reached_goal=False,
        action_status=6,
    )

    executor._validate_pear_close_result(plan, step, result)

    assert executor.node.logger.errors == []


def test_release_gripper_rejects_structured_stalled_not_open_result():
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.gripper_controller = SimpleNamespace(
        send_gripper_command=lambda position: gripper_result(
            accepted=True,
            target_position=position,
            actual_position=0.740,
            stalled=True,
            reached_goal=False,
            action_status=6,
        )
    )

    with pytest.raises(GripperCommandError, match="open command"):
        executor.execute_step(
            PlanStep(
                name="open_gripper_to_release",
                action="gripper",
                gripper_position=0.0,
            )
        )


def test_release_failure_stops_before_retreat():
    executor = object.__new__(ArmMotionExecutor)
    executor.node = DummyNode()
    executor.gripper_controller = SimpleNamespace(
        send_gripper_command=lambda position: gripper_result(
            accepted=True,
            target_position=position,
            actual_position=0.740,
            stalled=True,
            reached_goal=False,
            action_status=6,
        )
    )
    executed_holds = []
    executor.hold_pose = lambda pose, duration: executed_holds.append(duration)
    plan = SimpleNamespace(
        grasp_pose_6d=np.zeros(6),
        steps=[
            PlanStep(
                name="open_gripper_to_release",
                action="gripper",
                gripper_position=0.0,
            ),
            PlanStep(
                name="retreat_from_drop",
                action="move",
                mode="cartesian",
                start_pose_6d=np.zeros(6),
                end_pose_6d=np.ones(6),
            ),
        ],
    )

    with pytest.raises(GripperCommandError):
        executor.execute_plan(plan)

    assert executed_holds == []


def test_debug_stop_after_lift_skips_transfer_and_release(monkeypatch):
    executor = object.__new__(ArmMotionExecutor)
    executed = []
    debug_stages = []
    executor.execute_step = lambda step: executed.append(step.name)
    executor._log_grasp_debug = lambda plan, stage: debug_stages.append(stage)
    executor.converge_at_final_grasp = lambda pose, **kwargs: pose
    executor.debug_stop_reached = False
    monkeypatch.setattr(
        executor_module,
        "GRASP_DEBUG_STOP_AFTER_LIFT",
        True,
        raising=False,
    )
    monkeypatch.setattr(
        executor_module,
        "GRASP_DEBUG_STOP_AT_GRASP",
        False,
        raising=False,
    )
    monkeypatch.setattr(
        executor_module,
        "GRASP_DEBUG_STOP_AFTER_CLOSE",
        False,
        raising=False,
    )
    monkeypatch.setattr(
        executor_module,
        "GRASP_DEBUG_STOP_BEFORE_RELEASE",
        False,
        raising=False,
    )
    plan = SimpleNamespace(
        grasp_pose_6d=np.zeros(6),
        steps=[
            PlanStep(name="lift_after_grasp", action="move", mode="cartesian"),
            PlanStep(
                name="hold_after_lift",
                action="hold",
                duration_sec=0.01,
            ),
            PlanStep(name="transfer_to_drop_high", action="move", mode="moveit"),
            PlanStep(
                name="open_gripper_to_release",
                action="gripper",
                gripper_position=0.0,
            ),
        ],
    )

    executor.execute_plan(plan)

    assert executed == ["lift_after_grasp", "hold_after_lift"]
    assert debug_stages == ["after_lift"]
    assert executor.debug_stop_reached is True

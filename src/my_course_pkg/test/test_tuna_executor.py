from types import SimpleNamespace

import numpy as np
import pytest

from my_course_pkg.grasp.tuna_errors import TunaErrorCode, TunaGraspError
from my_course_pkg.grasp.tuna_executor import TunaExecutionCoordinator
from my_course_pkg.grasp.tuna_finalizer import (
    MoveItPoseValidation,
    TunaDependencyUnavailable,
    TunaFinalizer,
)
from my_course_pkg.grasp.trajectory_planner import (
    PickPlacePlan,
    PlanStep,
    build_tuna_preclamp_plan,
)
from my_course_pkg.grasp.tuna_roll_grasp import (
    CalibrationPoint,
    CalibrationTable,
    FrozenPivotContract,
    QposSample,
    StableTunaObservation,
    TunaBoundsSample,
    interpolate_gripper_geometry,
)


def calibration():
    transform = np.eye(4)
    hull = np.array([[0.0, 0.0, 0.0], [0.001, 0.0, 0.0]])
    return CalibrationTable(
        source_sha256="b" * 64,
        max_interpolation_error_m=0.0001,
        points=(
            CalibrationPoint(0.0, 0.100, transform, hull),
            CalibrationPoint(1.0, 0.010, transform, hull),
        ),
    )


def pivot(table, qpos=0.4):
    geometry = interpolate_gripper_geometry(table, qpos)
    return FrozenPivotContract(
        calibration_sha256=table.source_sha256,
        sample_timestamps=(1.0, 2.0, 3.0),
        qpos_peak_to_peak_rad=0.0,
        measured_qpos=qpos,
        measured_pad_gap_m=geometry.pad_gap_m,
        T_world_tcp_preclamp=np.eye(4),
        T_tcp_pivot_frozen=geometry.T_tcp_lower_pad_contact,
        T_world_pivot=geometry.T_tcp_lower_pad_contact,
    )


def qpos_samples(boundary, value):
    return tuple(
        QposSample(float(boundary + 0.01 * (index + 1)), float(value))
        for index in range(3)
    )


class FakeRuntime:
    def __init__(self, qpos_values=()):
        self.node = SimpleNamespace()
        self.qpos_values = list(qpos_values)
        self.micro_poses = []
        self.steps = []
        self.holds = []
        self.bounds_source = 10.0

    def collect_tuna_qpos_samples(self, *, required_count, command_boundary):
        assert required_count == 3
        value = self.qpos_values.pop(0) if self.qpos_values else 0.4
        return qpos_samples(command_boundary, value)

    def execute_tuna_micro_pose(self, pose, step_name):
        self.micro_poses.append((step_name, np.asarray(pose).copy()))
        return True

    def execute_tuna_step(self, step):
        self.steps.append(step.name)
        return True

    def current_tuna_joint_state(self):
        return ("joint_a",), (0.0,)

    def collect_tuna_bounds_samples(self, *, required_count, reception_boundary):
        result = []
        for index in range(required_count):
            self.bounds_source += 1.0
            result.append(
                TunaBoundsSample(
                    object_name="tuna_fish_can",
                    frame_id="world",
                    source_timestamp=self.bounds_source,
                    received_monotonic=reception_boundary + 0.01 * (index + 1),
                    center=np.array([0.4, -0.2, 0.015]),
                    size=np.array([0.085, 0.084, 0.030]),
                )
            )
        return tuple(result)

    def hold_tuna_failure(self, target):
        self.holds.append(target)

    def log_tuna_event(self, event, details):
        pass


def generation_plan(object_name="tuna_fish_can"):
    pose = np.zeros(6)
    return PickPlacePlan(
        start_pose_6d=pose.copy(),
        pre_grasp_pose_6d=pose.copy(),
        grasp_pose_6d=pose.copy(),
        drop_high_pose_6d=pose.copy(),
        drop_pose_6d=pose.copy(),
        steps=[],
        debug_info={"object_name": object_name},
    )


def test_generation_stage_is_guaranteed_zero_command(monkeypatch):
    monkeypatch.setenv("GRASP_TUNA_DEBUG_STOP_AFTER", "generation")
    runtime = FakeRuntime()
    outcome = TunaExecutionCoordinator(runtime, calibration()).execute(
        generation_plan()
    )
    assert outcome.stopped_after == "generation"
    assert outcome.physical_command_count == 0
    assert runtime.steps == []
    assert runtime.micro_poses == []


def test_coordinator_rejects_non_tuna_without_command(monkeypatch):
    monkeypatch.setenv("GRASP_TUNA_DEBUG_STOP_AFTER", "generation")
    runtime = FakeRuntime()
    with pytest.raises(TunaGraspError) as exc_info:
        TunaExecutionCoordinator(runtime, calibration()).execute(
            generation_plan("pear")
        )
    assert exc_info.value.code is TunaErrorCode.CONFIGURATION
    assert runtime.steps == []


def test_roll_checks_aperture_before_first_and_after_each_microsegment(monkeypatch):
    monkeypatch.setenv("GRASP_TUNA_DEBUG_STOP_AFTER", "none")
    monkeypatch.setenv("GRASP_TUNA_PRECLAMP_POSITION", "0.4")
    table = calibration()
    runtime = FakeRuntime(qpos_values=(0.4, 0.5))
    coordinator = TunaExecutionCoordinator(runtime, table)
    coordinator.pivot_contract = pivot(table)
    step = PlanStep(
        name="roll_tuna_segment_01_of_03",
        action="move",
        mode="cartesian",
    )
    poses = (np.zeros(6), np.ones(6) * 0.001)

    with pytest.raises(TunaGraspError) as exc_info:
        coordinator._dispatch_micro_step(step, poses)

    assert exc_info.value.code is TunaErrorCode.PIVOT_INVALIDATED
    assert len(runtime.micro_poses) == 1
    assert coordinator.command_count == 1


def test_runtime_command_cap_stops_before_next_microcommand(monkeypatch):
    monkeypatch.setenv("GRASP_TUNA_DEBUG_STOP_AFTER", "none")
    monkeypatch.setenv("GRASP_TUNA_PRECLAMP_POSITION", "0.4")
    monkeypatch.setenv("GRASP_TUNA_MAX_PHYSICAL_COMMANDS", "1")
    table = calibration()
    runtime = FakeRuntime(qpos_values=(0.4, 0.4))
    coordinator = TunaExecutionCoordinator(runtime, table)
    coordinator.pivot_contract = pivot(table)
    step = PlanStep(
        name="roll_tuna_segment_01_of_03",
        action="move",
        mode="cartesian",
    )
    with pytest.raises(TunaGraspError) as exc_info:
        coordinator._dispatch_micro_step(step, (np.zeros(6), np.ones(6) * 0.001))
    assert exc_info.value.code is TunaErrorCode.COMMAND_BUDGET
    assert len(runtime.micro_poses) == 1
    assert coordinator.command_count == 1


class OneTransientBackend:
    def __init__(self):
        self.failed = False

    def validate_pose(self, pose, joint_names, joint_positions, stage):
        if not self.failed:
            self.failed = True
            raise TunaDependencyUnavailable("temporary service loss")
        return MoveItPoseValidation(
            success=True,
            joint_names=tuple(joint_names),
            joint_positions=tuple(joint_positions),
        )


def test_finalization_retries_one_transient_then_commits_atomically(monkeypatch):
    monkeypatch.setenv("GRASP_TUNA_DEBUG_STOP_AFTER", "none")
    monkeypatch.setenv("GRASP_TUNA_PRECLAMP_POSITION", "0.4")
    table = calibration()
    runtime = FakeRuntime(qpos_values=(0.4, 0.4))
    plan = build_tuna_preclamp_plan(
        start_pose_6d=np.zeros(6),
        pregrasp_pose_6d=np.array([0.0, 0.0, 0.1, 0.0, 0.0, 0.0]),
        contact_support_pose_6d=np.zeros(6),
        preclamp_position=0.4,
        debug_info={
            "tuna_calibration_sha256": table.source_sha256,
            "execution_mode": "lift_return",
        },
    )
    backend = OneTransientBackend()
    finalizer = TunaFinalizer(runtime.node, backend_factory=lambda: backend)
    sleeps = []
    coordinator = TunaExecutionCoordinator(
        runtime,
        table,
        finalizer=finalizer,
        sleep=sleeps.append,
    )
    coordinator.plan = plan
    coordinator.pivot_contract = pivot(table)
    coordinator.preclamp_observation = StableTunaObservation(
        source_timestamps=(1.0, 2.0),
        received_monotonic=(1.0, 2.0),
        center=np.array([0.4, -0.2, 0.015]),
        size=np.array([0.085, 0.084, 0.030]),
    )
    coordinator.last_bounds_source_timestamp = 2.0

    committed = coordinator._finalize_once_or_retry(
        plan, np.array([0.0, 1.0, 0.0])
    )

    assert sleeps == [0.5]
    assert committed.plan_revision == 1
    assert committed.deferred_tuna_suffix is None
    assert committed.debug_info["tuna_suffix_commit"]["attempt_count"] == 2


def test_fresh_rolled_observation_accepts_nonflat_aabb():
    runtime = FakeRuntime()
    boundary = 100.0
    samples = tuple(
        TunaBoundsSample(
            object_name="tuna_fish_can",
            frame_id="world",
            source_timestamp=float(index + 1),
            received_monotonic=boundary + index + 1,
            center=np.array([0.4, -0.2, 0.04]),
            size=np.array([0.08, 0.05, 0.072]),
        )
        for index in range(2)
    )
    from my_course_pkg.grasp.tuna_roll_grasp import validate_tuna_observation_samples

    observed = validate_tuna_observation_samples(
        samples,
        required_count=2,
        source_boundary=0.0,
        reception_boundary=boundary,
        stability_tolerance_m=0.005,
    )
    assert observed.size[2] == pytest.approx(0.072)

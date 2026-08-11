import numpy as np
import pytest

from my_course_pkg.grasp.tuna_errors import TunaErrorCode, TunaGraspError
from my_course_pkg.grasp.tuna_finalizer import (
    MoveItPoseValidation,
    TunaFinalizer,
    finalize_tuna_post_preclamp,
)
from my_course_pkg.grasp.trajectory_planner import build_tuna_preclamp_plan
from my_course_pkg.grasp.tuna_roll_grasp import FrozenPivotContract


def transform(translation=(0.0, 0.0, 0.0)):
    result = np.eye(4, dtype=float)
    result[:3, 3] = translation
    return result


def pivot_contract():
    T_world_tcp = transform((0.4, -0.2, 0.75))
    T_tcp_pivot = transform((0.0, 0.0, 0.08))
    return FrozenPivotContract(
        calibration_sha256="a" * 64,
        sample_timestamps=(1.1, 1.2, 1.3),
        qpos_peak_to_peak_rad=0.001,
        measured_qpos=0.5,
        measured_pad_gap_m=0.035,
        T_world_tcp_preclamp=T_world_tcp,
        T_tcp_pivot_frozen=T_tcp_pivot,
        T_world_pivot=T_world_tcp @ T_tcp_pivot,
    )


def tuna_plan():
    return build_tuna_preclamp_plan(
        start_pose_6d=np.array([0.2, -0.1, 0.9, 0.0, 0.0, 0.0]),
        pregrasp_pose_6d=np.array([0.4, -0.2, 0.85, 0.0, 0.0, 0.0]),
        contact_support_pose_6d=np.array([0.4, -0.2, 0.75, 0.0, 0.0, 0.0]),
        preclamp_position=0.5,
    )


def prepare_kwargs(plan=None):
    return {
        "plan": tuna_plan() if plan is None else plan,
        "pivot_contract": pivot_contract(),
        "current_joint_names": ("joint_1",),
        "current_joint_positions": (0.0,),
        "roll_axis_world": (0.0, 1.0, 0.0),
        "attempt": 1,
        "evidence_after_timestamp": 10.0,
        "execution_mode": "lift_return",
    }


class SuccessfulBackend:
    def __init__(self):
        self.calls = []

    def validate_pose(self, pose, joint_names, joint_positions, stage):
        self.calls.append(
            (np.array(pose, copy=True), tuple(joint_names), tuple(joint_positions), stage)
        )
        next_position = float(joint_positions[0]) + 0.01
        return MoveItPoseValidation(
            success=True,
            joint_names=("joint_1",),
            joint_positions=(next_position,),
        )


def test_finalizer_is_lazy_and_non_tuna_rejects_before_client_construction():
    constructed = []
    finalizer = TunaFinalizer(
        node=object(),
        backend_factory=lambda: constructed.append(True),
    )
    plan = tuna_plan()
    plan.debug_info["object_name"] = "tomato_soup_can"

    with pytest.raises(TunaGraspError) as exc_info:
        finalize_tuna_post_preclamp(finalizer, **prepare_kwargs(plan))

    assert exc_info.value.code is TunaErrorCode.CONFIGURATION
    assert constructed == []


def test_finalizer_seeds_every_ik_from_previous_validated_state_and_never_mutates_plan():
    backend = SuccessfulBackend()
    finalizer = TunaFinalizer(node=object(), backend_factory=lambda: backend)
    plan = tuna_plan()
    original_names = [step.name for step in plan.steps]

    result = finalize_tuna_post_preclamp(finalizer, **prepare_kwargs(plan))

    assert result.validated_pose_count == len(backend.calls)
    assert result.validated_pose_count > 20
    assert backend.calls[0][2] == (0.0,)
    for index, call in enumerate(backend.calls[1:], start=1):
        assert call[2] == pytest.approx((0.01 * index,))
    assert result.final_joint_positions == pytest.approx(
        (0.01 * result.validated_pose_count,)
    )
    assert len(result.pivot_contract_sha256) == 64
    assert [step.name for step in plan.steps] == original_names
    assert plan.deferred_tuna_suffix is not None


def test_prepared_suffix_debug_metadata_is_immutable_before_commit():
    backend = SuccessfulBackend()
    result = TunaFinalizer(
        node=object(),
        backend_factory=lambda: backend,
    ).prepare(**prepare_kwargs())

    with pytest.raises(TypeError):
        result.prepared_suffix.debug_updates["new"] = "mutation"
    first_roll = result.prepared_suffix.debug_updates["tuna_micro_waypoints"][
        "roll_tuna_segment_01_of_03"
    ][0]
    with pytest.raises(ValueError):
        first_roll[0] = 999.0


def test_result_free_timeout_is_the_only_retryable_failure_class():
    class TimeoutBackend:
        def validate_pose(self, *_args):
            raise TimeoutError("no response")

    finalizer = TunaFinalizer(
        node=object(),
        backend_factory=lambda: TimeoutBackend(),
    )
    with pytest.raises(TunaGraspError) as exc_info:
        finalizer.prepare(**prepare_kwargs())

    error = exc_info.value
    assert error.code is TunaErrorCode.DEPENDENCY_TRANSIENT
    assert error.retryable is True
    assert error.diagnostics["result_received"] is False
    assert error.diagnostics["robot_command_issued"] is False


def test_semantic_no_ik_response_is_nonretryable():
    class NoIkBackend:
        def validate_pose(self, *_args):
            return MoveItPoseValidation(
                success=False,
                semantic_response_received=True,
                reason="no_ik",
            )

    finalizer = TunaFinalizer(node=object(), backend_factory=lambda: NoIkBackend())
    with pytest.raises(TunaGraspError) as exc_info:
        finalizer.prepare(**prepare_kwargs())

    assert exc_info.value.code is TunaErrorCode.PLANNING
    assert exc_info.value.retryable is False
    assert exc_info.value.diagnostics["semantic_response_received"] is True


def test_collision_response_is_nonretryable_and_keeps_link_evidence():
    class CollisionBackend:
        def validate_pose(self, *_args):
            return MoveItPoseValidation(
                success=False,
                collision_links=("wrist_3_link", "table"),
                reason="state_invalid",
            )

    finalizer = TunaFinalizer(
        node=object(),
        backend_factory=lambda: CollisionBackend(),
    )
    with pytest.raises(TunaGraspError) as exc_info:
        finalizer.prepare(**prepare_kwargs())

    assert exc_info.value.code is TunaErrorCode.COLLISION
    assert exc_info.value.retryable is False
    assert exc_info.value.diagnostics["collision_links"] == [
        "wrist_3_link",
        "table",
    ]


def test_malformed_backend_result_is_semantic_failure_not_transient():
    class MalformedBackend:
        def validate_pose(self, *_args):
            return object()

    finalizer = TunaFinalizer(
        node=object(),
        backend_factory=lambda: MalformedBackend(),
    )
    with pytest.raises(TunaGraspError) as exc_info:
        finalizer.prepare(**prepare_kwargs())

    assert exc_info.value.code is TunaErrorCode.PLANNING
    assert exc_info.value.retryable is False

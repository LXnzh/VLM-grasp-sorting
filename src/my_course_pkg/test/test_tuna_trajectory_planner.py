import numpy as np
import pytest

from my_course_pkg.grasp.trajectory_planner import (
    DeferredTunaSuffix,
    build_prepared_tuna_suffix,
    build_tuna_preclamp_plan,
    commit_prepared_tuna_suffix,
)
from my_course_pkg.grasp.tuna_roll_grasp import (
    FinalizationCommitMetadata,
    FrozenPivotContract,
)


def transform(translation=(0.0, 0.0, 0.0)):
    result = np.eye(4, dtype=float)
    result[:3, 3] = translation
    return result


def pivot_contract():
    T_world_tcp = transform((0.40, -0.20, 0.75))
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


def prefix_plan(final_roll_angle_deg=30.0):
    return build_tuna_preclamp_plan(
        start_pose_6d=np.array([0.2, -0.1, 0.9, 0.0, 0.0, 0.0]),
        pregrasp_pose_6d=np.array([0.4, -0.2, 0.85, 0.0, 0.0, 0.0]),
        contact_support_pose_6d=np.array([0.4, -0.2, 0.75, 0.0, 0.0, 0.0]),
        preclamp_position=0.5,
        final_roll_angle_deg=final_roll_angle_deg,
    )


def test_initial_tuna_plan_stops_at_typed_nonexecutable_deferred_suffix():
    plan = prefix_plan()
    assert [step.name for step in plan.steps] == [
        "open_gripper_before_tuna_approach",
        "move_to_tuna_pregrasp",
        "approach_tuna_contact_support",
        "preclamp_tuna",
        "observe_tuna_after_preclamp",
    ]
    assert [step.action for step in plan.steps] == [
        "gripper",
        "move",
        "move",
        "gripper",
        "observe_tuna_bounds",
    ]
    assert isinstance(plan.deferred_tuna_suffix, DeferredTunaSuffix)
    assert not any("roll_tuna" in step.name for step in plan.steps)
    assert plan.debug_info["object_name"] == "tuna_fish_can"
    assert plan.debug_info["tuna_hold_contract_timeline"] == [
        {
            "target": "preclamp",
            "starts_after": "preclamp_tuna",
            "ends_before": "close_tuna_after_roll",
        },
        {
            "target": "full_close",
            "starts_after": "close_tuna_after_roll",
            "ends_after": "normal_lift",
        },
    ]


def test_tuna_prefix_refuses_unqualified_or_endpoint_preclamp():
    for value in (0.0, 0.79, np.nan):
        with pytest.raises(ValueError, match="preclamp"):
            build_tuna_preclamp_plan(
                start_pose_6d=np.zeros(6),
                pregrasp_pose_6d=np.zeros(6),
                contact_support_pose_6d=np.zeros(6),
                preclamp_position=value,
            )


def test_prepared_suffix_has_indexed_roll_close_test_and_normal_lift_order():
    plan = prefix_plan()
    prepared = build_prepared_tuna_suffix(
        plan=plan,
        pivot_contract=pivot_contract(),
        roll_axis_world=np.array([0.0, 1.0, 0.0]),
        execution_mode="lift_return",
    )
    names = [step.name for step in prepared.steps]
    assert names[:6] == [
        "roll_tuna_segment_01_of_03",
        "observe_tuna_after_roll_segment_01_of_03",
        "roll_tuna_segment_02_of_03",
        "observe_tuna_after_roll_segment_02_of_03",
        "roll_tuna_segment_03_of_03",
        "observe_tuna_after_roll_segment_03_of_03",
    ]
    assert names[6:11] == [
        "close_tuna_after_roll",
        "hold_tuna_after_close",
        "observe_tuna_after_close",
        "test_lift_tuna_30mm",
        "observe_tuna_after_test_lift",
    ]
    normal_moves = [name for name in names if name.startswith("normal_lift_tuna_segment_")]
    normal_observations = [
        name
        for name in names
        if name.startswith("observe_tuna_after_normal_lift_segment_")
    ]
    assert len(normal_moves) == len(normal_observations) == 4
    assert prepared.physical_command_count <= 96
    assert plan.deferred_tuna_suffix is not None
    assert len(plan.steps) == 5


def test_roll_and_lift_micro_waypoints_obey_limits_and_keep_rolled_orientation():
    prepared = build_prepared_tuna_suffix(
        plan=prefix_plan(),
        pivot_contract=pivot_contract(),
        roll_axis_world=np.array([0.0, 1.0, 0.0]),
        execution_mode="lift_return",
    )
    debug = prepared.debug_updates
    micro = debug["tuna_micro_waypoints"]
    for index in range(1, 4):
        assert len(micro[f"roll_tuna_segment_{index:02d}_of_03"]) == 4
    assert len(micro["test_lift_tuna_30mm"]) == 3

    test_poses = micro["test_lift_tuna_30mm"]
    np.testing.assert_allclose(test_poses[0][3:], test_poses[-1][3:], atol=1e-12)
    z_deltas = np.diff([pivot_contract().T_world_tcp_preclamp[2, 3]] + [pose[2] for pose in test_poses])
    # The first delta includes the roll's pivot-induced translation; check only
    # separately generated lift increments after the first point.
    assert np.max(z_deltas[1:]) <= 0.010 + 1e-12

    for name, poses in micro.items():
        if name.startswith("normal_lift_tuna_segment_"):
            assert len(poses) >= 1
            assert np.max(np.diff([pose[2] for pose in poses])) <= 0.010 + 1e-12


def test_configurable_final_roll_angle_keeps_segment_three_semantics():
    plan = prefix_plan(final_roll_angle_deg=27.0)
    prepared = build_prepared_tuna_suffix(
        plan=plan,
        pivot_contract=pivot_contract(),
        roll_axis_world=[0.0, 1.0, 0.0],
        execution_mode="lift_return",
    )
    observation = prepared.debug_updates["tuna_observations"][
        "observe_tuna_after_roll_segment_03_of_03"
    ]
    assert observation["stage"] == "roll_segment_3"
    assert observation["end_angle_deg"] == pytest.approx(27.0)


def test_command_budget_fails_before_commit_or_motion():
    plan = prefix_plan()
    with pytest.raises(ValueError, match="command budget"):
        build_prepared_tuna_suffix(
            plan=plan,
            pivot_contract=pivot_contract(),
            roll_axis_world=[0.0, 1.0, 0.0],
            execution_mode="lift_return",
            max_physical_commands=10,
        )
    assert plan.deferred_tuna_suffix is not None
    assert len(plan.steps) == 5


def test_suffix_commit_is_single_use_atomic_and_does_not_mutate_input():
    plan = prefix_plan()
    prepared = build_prepared_tuna_suffix(
        plan=plan,
        pivot_contract=pivot_contract(),
        roll_axis_world=[0.0, 1.0, 0.0],
        execution_mode="lift_return",
    )
    metadata = FinalizationCommitMetadata(
        suffix_generation_id="generation-1",
        attempt_count=1,
        pivot_contract_sha256="b" * 64,
        plan_revision=0,
    )
    committed = commit_prepared_tuna_suffix(plan, prepared, metadata)

    assert plan.deferred_tuna_suffix is not None
    assert len(plan.steps) == 5
    assert committed.deferred_tuna_suffix is None
    assert committed.plan_revision == 1
    assert len(committed.steps) == len(plan.steps) + len(prepared.steps)
    assert committed.debug_info["tuna_suffix_commit"]["suffix_generation_id"] == "generation-1"

    with pytest.raises(ValueError, match="already committed|absent"):
        commit_prepared_tuna_suffix(committed, prepared, metadata)

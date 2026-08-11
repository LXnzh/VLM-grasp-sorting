import numpy as np
import pytest

from my_course_pkg.grasp.config import (
    APPROACH_DIST,
    DROP_POSITION,
    GRASP_LIFT_HEIGHT,
    GRASP_LIFT_HOLD_SEC,
    GRASP_RETURN_RELEASE_CLEARANCE_M,
    RELEASE_PRE_OPEN_HOLD_SEC,
)
from my_course_pkg.grasp import config as grasp_config_module
from my_course_pkg.grasp import trajectory_planner as trajectory_planner_module
from my_course_pkg.grasp.trajectory_planner import plan_safe_pick_place_steps


def test_execution_mode_defaults_to_lift_return(monkeypatch):
    monkeypatch.delenv("GRASP_EXECUTION_MODE", raising=False)

    assert grasp_config_module.read_grasp_execution_mode() == "lift_return"


def test_execution_mode_rejects_unknown_value(monkeypatch):
    monkeypatch.setenv("GRASP_EXECUTION_MODE", "unknown")

    with pytest.raises(ValueError, match="GRASP_EXECUTION_MODE"):
        grasp_config_module.read_grasp_execution_mode()


def test_return_release_clearance_defaults_to_two_centimeters(monkeypatch):
    monkeypatch.delenv("GRASP_RETURN_RELEASE_CLEARANCE_M", raising=False)

    assert grasp_config_module.read_return_release_clearance_m() == pytest.approx(
        0.02
    )


def test_vertical_grasp_z_offset_defaults_to_zero(monkeypatch):
    monkeypatch.delenv("GRASP_VERTICAL_Z_OFFSET", raising=False)

    assert grasp_config_module.read_vertical_grasp_z_offset() == pytest.approx(0.0)


def test_vertical_grasp_z_offset_honors_environment_override(monkeypatch):
    monkeypatch.setenv("GRASP_VERTICAL_Z_OFFSET", "0.012")

    assert grasp_config_module.read_vertical_grasp_z_offset() == pytest.approx(
        0.012
    )


@pytest.mark.parametrize("invalid_value", ["nan", "inf", "-inf"])
def test_vertical_grasp_z_offset_rejects_non_finite_value(
    monkeypatch,
    invalid_value,
):
    monkeypatch.setenv("GRASP_VERTICAL_Z_OFFSET", invalid_value)

    with pytest.raises(ValueError, match="GRASP_VERTICAL_Z_OFFSET"):
        grasp_config_module.read_vertical_grasp_z_offset()


def test_vertical_min_tcp_above_target_bottom_defaults_to_eight_millimeters(
    monkeypatch,
):
    monkeypatch.delenv(
        "GRASP_VERTICAL_MIN_TCP_ABOVE_TARGET_BOTTOM_M",
        raising=False,
    )

    assert (
        grasp_config_module.read_vertical_min_tcp_above_target_bottom_m()
        == pytest.approx(0.008)
    )


def test_vertical_min_tcp_above_target_bottom_honors_environment_override(
    monkeypatch,
):
    monkeypatch.setenv("GRASP_VERTICAL_MIN_TCP_ABOVE_TARGET_BOTTOM_M", "0.012")

    assert (
        grasp_config_module.read_vertical_min_tcp_above_target_bottom_m()
        == pytest.approx(0.012)
    )


@pytest.mark.parametrize("invalid_value", ["nan", "inf", "-inf", "-0.001"])
def test_vertical_min_tcp_above_target_bottom_rejects_invalid_value(
    monkeypatch,
    invalid_value,
):
    monkeypatch.setenv(
        "GRASP_VERTICAL_MIN_TCP_ABOVE_TARGET_BOTTOM_M",
        invalid_value,
    )

    with pytest.raises(
        ValueError,
        match="GRASP_VERTICAL_MIN_TCP_ABOVE_TARGET_BOTTOM_M",
    ):
        grasp_config_module.read_vertical_min_tcp_above_target_bottom_m()


def test_top_down_grasp_z_offset_defaults_to_zero(monkeypatch):
    monkeypatch.delenv("GRASP_TOP_DOWN_Z_OFFSET", raising=False)

    assert grasp_config_module.read_top_down_grasp_z_offset() == pytest.approx(
        0.0
    )


def test_top_down_grasp_z_offset_honors_environment_override(monkeypatch):
    monkeypatch.setenv("GRASP_TOP_DOWN_Z_OFFSET", "0.012")

    assert grasp_config_module.read_top_down_grasp_z_offset() == pytest.approx(
        0.012
    )


@pytest.mark.parametrize("invalid_value", ["nan", "inf", "-inf"])
def test_top_down_grasp_z_offset_rejects_non_finite_value(
    monkeypatch,
    invalid_value,
):
    monkeypatch.setenv("GRASP_TOP_DOWN_Z_OFFSET", invalid_value)

    with pytest.raises(ValueError, match="GRASP_TOP_DOWN_Z_OFFSET"):
        grasp_config_module.read_top_down_grasp_z_offset()


def test_hammer_balance_configuration_defaults(monkeypatch):
    for name in (
        "GRASP_HAMMER_BALANCE_X",
        "GRASP_HAMMER_BALANCE_Y",
        "GRASP_HAMMER_BALANCE_Z",
        "GRASP_HAMMER_BALANCE_WEIGHT",
        "GRASP_HAMMER_MAX_TOP_DOWN_ANGLE_DEG",
    ):
        monkeypatch.delenv(name, raising=False)

    np.testing.assert_allclose(
        grasp_config_module.read_hammer_grasp_balance_point(),
        [-0.030227, -0.009931, 0.015676],
    )
    assert grasp_config_module.read_hammer_grasp_balance_weight() == pytest.approx(
        250.0
    )
    assert grasp_config_module.read_hammer_max_top_down_angle_deg() == pytest.approx(
        10.0
    )


def test_hammer_balance_configuration_honors_environment(monkeypatch):
    monkeypatch.setenv("GRASP_HAMMER_BALANCE_X", "0.01")
    monkeypatch.setenv("GRASP_HAMMER_BALANCE_Y", "0.02")
    monkeypatch.setenv("GRASP_HAMMER_BALANCE_Z", "0.03")
    monkeypatch.setenv("GRASP_HAMMER_BALANCE_WEIGHT", "300.0")
    monkeypatch.setenv("GRASP_HAMMER_MAX_TOP_DOWN_ANGLE_DEG", "8.0")

    np.testing.assert_allclose(
        grasp_config_module.read_hammer_grasp_balance_point(),
        [0.01, 0.02, 0.03],
    )
    assert grasp_config_module.read_hammer_grasp_balance_weight() == pytest.approx(
        300.0
    )
    assert grasp_config_module.read_hammer_max_top_down_angle_deg() == pytest.approx(
        8.0
    )


@pytest.mark.parametrize(
    "name",
    [
        "GRASP_HAMMER_BALANCE_X",
        "GRASP_HAMMER_BALANCE_Y",
        "GRASP_HAMMER_BALANCE_Z",
    ],
)
def test_hammer_balance_point_rejects_non_finite_value(monkeypatch, name):
    monkeypatch.setenv(name, "nan")

    with pytest.raises(ValueError, match="GRASP_HAMMER_BALANCE_X/Y/Z"):
        grasp_config_module.read_hammer_grasp_balance_point()


@pytest.mark.parametrize("invalid_value", ["-0.001", "nan", "inf"])
def test_hammer_balance_weight_rejects_invalid_value(
    monkeypatch,
    invalid_value,
):
    monkeypatch.setenv("GRASP_HAMMER_BALANCE_WEIGHT", invalid_value)

    with pytest.raises(ValueError, match="GRASP_HAMMER_BALANCE_WEIGHT"):
        grasp_config_module.read_hammer_grasp_balance_weight()


@pytest.mark.parametrize("invalid_value", ["-0.001", "180.001", "nan", "inf"])
def test_hammer_max_top_down_angle_rejects_invalid_value(
    monkeypatch,
    invalid_value,
):
    monkeypatch.setenv("GRASP_HAMMER_MAX_TOP_DOWN_ANGLE_DEG", invalid_value)

    with pytest.raises(
        ValueError,
        match="GRASP_HAMMER_MAX_TOP_DOWN_ANGLE_DEG",
    ):
        grasp_config_module.read_hammer_max_top_down_angle_deg()


def test_final_approach_tolerance_defaults_to_18_mm(monkeypatch):
    monkeypatch.delenv(
        "GRASP_FINAL_APPROACH_POSITION_TOLERANCE_M",
        raising=False,
    )
    monkeypatch.setenv(
        "GRASP_VERTICAL_FINAL_APPROACH_POSITION_TOLERANCE_M",
        "0.001",
    )

    assert (
        grasp_config_module.read_final_approach_position_tolerance_m()
        == pytest.approx(0.018)
    )


def test_final_approach_tolerance_honors_environment_override(
    monkeypatch,
):
    monkeypatch.setenv(
        "GRASP_FINAL_APPROACH_POSITION_TOLERANCE_M",
        "0.021",
    )

    assert (
        grasp_config_module.read_final_approach_position_tolerance_m()
        == pytest.approx(0.021)
    )


@pytest.mark.parametrize("invalid_value", ["-0.001", "nan", "inf"])
def test_final_approach_tolerance_rejects_invalid_value(
    monkeypatch,
    invalid_value,
):
    monkeypatch.setenv(
        "GRASP_FINAL_APPROACH_POSITION_TOLERANCE_M",
        invalid_value,
    )

    with pytest.raises(
        ValueError,
        match="GRASP_FINAL_APPROACH_POSITION_TOLERANCE_M",
    ):
        grasp_config_module.read_final_approach_position_tolerance_m()


def test_vertical_approach_feedback_gate_defaults(monkeypatch):
    for name in (
        "GRASP_VERTICAL_APPROACH_XY_TOLERANCE_M",
        "GRASP_VERTICAL_DESCENT_XY_TOLERANCE_M",
        "GRASP_VERTICAL_APPROACH_Z_TOLERANCE_M",
        "GRASP_VERTICAL_APPROACH_WAYPOINT_MAX_DIST_M",
    ):
        monkeypatch.delenv(name, raising=False)

    assert (
        grasp_config_module.read_vertical_approach_xy_tolerance_m()
        == pytest.approx(0.0015)
    )
    assert (
        grasp_config_module.read_vertical_descent_xy_tolerance_m()
        == pytest.approx(0.0020)
    )
    assert (
        grasp_config_module.read_vertical_approach_z_tolerance_m()
        == pytest.approx(0.006)
    )
    assert (
        grasp_config_module.read_vertical_approach_waypoint_max_dist_m()
        == pytest.approx(0.005)
    )


@pytest.mark.parametrize(
    ("reader_name", "environment_name"),
    [
        (
            "read_vertical_approach_xy_tolerance_m",
            "GRASP_VERTICAL_APPROACH_XY_TOLERANCE_M",
        ),
        (
            "read_vertical_descent_xy_tolerance_m",
            "GRASP_VERTICAL_DESCENT_XY_TOLERANCE_M",
        ),
        (
            "read_vertical_approach_z_tolerance_m",
            "GRASP_VERTICAL_APPROACH_Z_TOLERANCE_M",
        ),
        (
            "read_vertical_approach_waypoint_max_dist_m",
            "GRASP_VERTICAL_APPROACH_WAYPOINT_MAX_DIST_M",
        ),
    ],
)
@pytest.mark.parametrize("invalid_value", ["0", "-0.001", "nan", "inf", "bad"])
def test_vertical_approach_feedback_gate_rejects_invalid_values(
    monkeypatch,
    reader_name,
    environment_name,
    invalid_value,
):
    monkeypatch.setenv(environment_name, invalid_value)

    with pytest.raises(ValueError, match=environment_name):
        getattr(grasp_config_module, reader_name)()


def test_vertical_reanchor_calibration_defaults(monkeypatch):
    for name in (
        "GRASP_VERTICAL_REANCHOR_MAX_OFFSET_M",
        "GRASP_VERTICAL_REANCHOR_MAX_ITERATIONS",
        "GRASP_VERTICAL_REANCHOR_SETTLE_SEC",
        "GRASP_VERTICAL_REANCHOR_SAMPLE_COUNT",
    ):
        monkeypatch.delenv(name, raising=False)

    assert grasp_config_module.read_vertical_reanchor_max_offset_m() == pytest.approx(
        0.03
    )
    assert grasp_config_module.read_vertical_reanchor_max_iterations() == 3
    assert grasp_config_module.read_vertical_reanchor_settle_sec() == pytest.approx(
        0.4
    )
    assert grasp_config_module.read_vertical_reanchor_sample_count() == 3


@pytest.mark.parametrize(
    ("reader_name", "environment_name"),
    [
        (
            "read_vertical_reanchor_max_offset_m",
            "GRASP_VERTICAL_REANCHOR_MAX_OFFSET_M",
        ),
        (
            "read_vertical_reanchor_settle_sec",
            "GRASP_VERTICAL_REANCHOR_SETTLE_SEC",
        ),
    ],
)
@pytest.mark.parametrize("invalid_value", ["0", "-0.1", "nan", "inf", "bad"])
def test_vertical_reanchor_float_configuration_rejects_invalid_values(
    monkeypatch,
    reader_name,
    environment_name,
    invalid_value,
):
    monkeypatch.setenv(environment_name, invalid_value)

    with pytest.raises(ValueError, match=environment_name):
        getattr(grasp_config_module, reader_name)()


@pytest.mark.parametrize("invalid_value", ["0", "-1", "1.5", "bad"])
def test_vertical_reanchor_max_iterations_rejects_invalid_values(
    monkeypatch,
    invalid_value,
):
    monkeypatch.setenv("GRASP_VERTICAL_REANCHOR_MAX_ITERATIONS", invalid_value)

    with pytest.raises(ValueError, match="GRASP_VERTICAL_REANCHOR_MAX_ITERATIONS"):
        grasp_config_module.read_vertical_reanchor_max_iterations()


@pytest.mark.parametrize("invalid_value", ["0", "1", "2", "4", "1.5", "bad"])
def test_vertical_reanchor_sample_count_rejects_invalid_values(
    monkeypatch,
    invalid_value,
):
    monkeypatch.setenv("GRASP_VERTICAL_REANCHOR_SAMPLE_COUNT", invalid_value)

    with pytest.raises(ValueError, match="GRASP_VERTICAL_REANCHOR_SAMPLE_COUNT"):
        grasp_config_module.read_vertical_reanchor_sample_count()


@pytest.mark.parametrize("invalid_value", ["-0.001", "nan", "inf"])
def test_return_release_clearance_rejects_invalid_value(
    monkeypatch,
    invalid_value,
):
    monkeypatch.setenv("GRASP_RETURN_RELEASE_CLEARANCE_M", invalid_value)

    with pytest.raises(ValueError, match="GRASP_RETURN_RELEASE_CLEARANCE_M"):
        grasp_config_module.read_return_release_clearance_m()


def test_side_grasp_vertical_margin_defaults_to_gripper_envelope(monkeypatch):
    monkeypatch.delenv("GRASP_SIDE_APPROACH_VERTICAL_MARGIN_M", raising=False)

    assert (
        grasp_config_module.read_side_grasp_approach_vertical_margin_m()
        == pytest.approx(0.12)
    )


def test_side_grasp_vertical_margin_honors_environment_override(monkeypatch):
    monkeypatch.setenv("GRASP_SIDE_APPROACH_VERTICAL_MARGIN_M", "0.08")

    assert (
        grasp_config_module.read_side_grasp_approach_vertical_margin_m()
        == pytest.approx(0.08)
    )


def test_lift_return_plan_lifts_holds_returns_releases_and_retreats():
    start_pose_6d = np.zeros(6)
    grasp_pose_6d = np.array([0.4, -0.2, 0.3, np.pi, 0.0, 0.5])

    plan = plan_safe_pick_place_steps(
        start_pose_6d=start_pose_6d,
        grasp_pose_6d=grasp_pose_6d,
        execution_mode="lift_return",
    )

    step_names = [step.name for step in plan.steps]
    assert step_names == [
        "open_gripper_before_approach",
        "move_to_pre_grasp",
        "approach_grasp",
        "close_gripper_at_grasp",
        "hold_after_close",
        "lift_after_grasp",
        "hold_after_lift",
        "return_to_grasp",
        "hold_before_release",
        "open_gripper_to_release",
        "hold_after_release",
        "retreat_after_release",
    ]
    assert "transfer_to_drop_high" not in step_names

    lift_step = plan.steps[5]
    hold_step = plan.steps[6]
    return_step = plan.steps[7]
    hold_before_release_step = plan.steps[8]
    hold_after_release_step = plan.steps[10]
    retreat_step = plan.steps[11]
    release_pose_6d = grasp_pose_6d.copy()
    release_pose_6d[2] += GRASP_RETURN_RELEASE_CLEARANCE_M
    np.testing.assert_allclose(lift_step.start_pose_6d, grasp_pose_6d)
    np.testing.assert_allclose(
        lift_step.end_pose_6d[:3],
        grasp_pose_6d[:3] + [0.0, 0.0, GRASP_LIFT_HEIGHT],
    )
    assert hold_step.duration_sec == pytest.approx(GRASP_LIFT_HOLD_SEC)
    assert hold_step.mode != "passive"
    np.testing.assert_allclose(return_step.start_pose_6d, lift_step.end_pose_6d)
    np.testing.assert_allclose(return_step.end_pose_6d, release_pose_6d)
    np.testing.assert_allclose(hold_before_release_step.pose_6d, release_pose_6d)
    np.testing.assert_allclose(hold_after_release_step.pose_6d, release_pose_6d)
    np.testing.assert_allclose(retreat_step.start_pose_6d, release_pose_6d)
    np.testing.assert_allclose(retreat_step.end_pose_6d, plan.pre_grasp_pose_6d)
    np.testing.assert_allclose(plan.drop_pose_6d, release_pose_6d)
    np.testing.assert_allclose(plan.drop_high_pose_6d, lift_step.end_pose_6d)


@pytest.mark.parametrize("clearance_m", [0.0, 0.035])
def test_lift_return_uses_configured_release_clearance(monkeypatch, clearance_m):
    monkeypatch.setattr(
        trajectory_planner_module,
        "GRASP_RETURN_RELEASE_CLEARANCE_M",
        clearance_m,
        raising=False,
    )
    grasp_pose_6d = np.array([0.4, -0.2, 0.3, np.pi, 0.0, 0.5])

    plan = plan_safe_pick_place_steps(
        start_pose_6d=np.zeros(6),
        grasp_pose_6d=grasp_pose_6d,
        execution_mode="lift_return",
    )

    return_step = next(step for step in plan.steps if step.name == "return_to_grasp")
    expected_release_pose_6d = grasp_pose_6d.copy()
    expected_release_pose_6d[2] += clearance_m
    np.testing.assert_allclose(return_step.end_pose_6d, expected_release_pose_6d)


def test_safe_place_ignores_return_release_clearance(monkeypatch):
    monkeypatch.setattr(
        trajectory_planner_module,
        "GRASP_RETURN_RELEASE_CLEARANCE_M",
        0.5,
        raising=False,
    )

    plan = plan_safe_pick_place_steps(
        start_pose_6d=np.zeros(6),
        grasp_pose_6d=np.array([0.4, -0.2, 0.3, np.pi, 0.0, 0.5]),
        execution_mode="safe_place",
    )

    assert "return_to_grasp" not in [step.name for step in plan.steps]
    np.testing.assert_allclose(plan.drop_pose_6d[:2], DROP_POSITION[:2])


def test_side_grasp_lifts_along_world_z_after_closing():
    start_pose_6d = np.zeros(6)
    side_grasp_pose_6d = np.array([0.4, 0.0, 0.2, 0.0, np.pi / 2.0, 0.0])

    plan = plan_safe_pick_place_steps(
        start_pose_6d=start_pose_6d,
        grasp_pose_6d=side_grasp_pose_6d,
        execution_mode="safe_place",
    )

    lift_step = plan.steps[5]

    assert lift_step.name == "lift_after_grasp"
    np.testing.assert_allclose(
        lift_step.end_pose_6d[:3],
        side_grasp_pose_6d[:3] + np.array([0.0, 0.0, GRASP_LIFT_HEIGHT]),
    )


def test_side_plan_uses_real_pregrasp_and_cartesian_final_approach():
    start_pose_6d = np.zeros(6)
    side_grasp_pose_6d = np.array([0.4, 0.0, 0.2, 0.0, np.pi / 2.0, 0.0])

    plan = plan_safe_pick_place_steps(
        start_pose_6d=start_pose_6d,
        grasp_pose_6d=side_grasp_pose_6d,
        direct_to_grasp=True,
        execution_mode="safe_place",
    )

    lift_step = next(step for step in plan.steps if step.name == "lift_after_grasp")
    approach_step = next(step for step in plan.steps if step.name == "approach_grasp")

    assert np.linalg.norm(
        plan.grasp_pose_6d[:3] - plan.pre_grasp_pose_6d[:3]
    ) == pytest.approx(APPROACH_DIST)
    assert approach_step.mode == "cartesian"
    np.testing.assert_allclose(approach_step.start_pose_6d, plan.pre_grasp_pose_6d)
    np.testing.assert_allclose(approach_step.end_pose_6d, plan.grasp_pose_6d)
    assert lift_step.mode == "cartesian"
    np.testing.assert_allclose(lift_step.start_pose_6d[:3], side_grasp_pose_6d[:3])
    np.testing.assert_allclose(
        lift_step.end_pose_6d[:3],
        side_grasp_pose_6d[:3] + np.array([0.0, 0.0, GRASP_LIFT_HEIGHT]),
    )


def test_plan_keeps_global_transfer_moves_in_moveit():
    start_pose_6d = np.zeros(6)
    side_grasp_pose_6d = np.array([0.4, 0.0, 0.2, 0.0, np.pi / 2.0, 0.0])

    plan = plan_safe_pick_place_steps(
        start_pose_6d=start_pose_6d,
        grasp_pose_6d=side_grasp_pose_6d,
        execution_mode="safe_place",
    )

    mode_by_name = {
        step.name: step.mode
        for step in plan.steps
        if step.action == "move"
    }
    hold_modes = [
        step.mode
        for step in plan.steps
        if step.action == "hold"
    ]

    assert mode_by_name["move_to_pre_grasp"] == "moveit"
    assert mode_by_name["transfer_to_drop_high"] == "moveit"
    assert mode_by_name["approach_grasp"] == "cartesian"
    assert mode_by_name["descend_to_drop"] == "cartesian"
    assert mode_by_name["retreat_from_drop"] == "cartesian"
    assert "passive" not in hold_modes


def test_direct_grasp_drop_descend_and_retreat_are_cartesian():
    start_pose_6d = np.zeros(6)
    side_grasp_pose_6d = np.array([0.4, 0.0, 0.2, 0.0, np.pi / 2.0, 0.0])

    plan = plan_safe_pick_place_steps(
        start_pose_6d=start_pose_6d,
        grasp_pose_6d=side_grasp_pose_6d,
        direct_to_grasp=True,
        execution_mode="safe_place",
    )

    mode_by_name = {
        step.name: step.mode
        for step in plan.steps
        if step.action == "move"
    }
    hold_after_lift = next(step for step in plan.steps if step.name == "hold_after_lift")

    assert mode_by_name["transfer_to_drop_high"] == "moveit"
    assert mode_by_name["descend_to_drop"] == "cartesian"
    assert mode_by_name["retreat_from_drop"] == "cartesian"
    assert hold_after_lift.mode != "passive"


def test_plan_uses_fixed_safe_drop_xy_and_grasp_release_height():
    start_pose_6d = np.zeros(6)
    side_grasp_pose_6d = np.array([0.4, 0.0, 0.2, 0.0, np.pi / 2.0, 0.0])

    plan = plan_safe_pick_place_steps(
        start_pose_6d=start_pose_6d,
        grasp_pose_6d=side_grasp_pose_6d,
        direct_to_grasp=True,
        execution_mode="safe_place",
    )

    descend_step = next(step for step in plan.steps if step.name == "descend_to_drop")

    np.testing.assert_allclose(plan.drop_pose_6d[:2], DROP_POSITION[:2])
    assert plan.drop_pose_6d[2] == pytest.approx(side_grasp_pose_6d[2])
    np.testing.assert_allclose(plan.drop_pose_6d[3:], side_grasp_pose_6d[3:])
    assert descend_step.end_pose_6d[2] == pytest.approx(side_grasp_pose_6d[2])


def test_fixed_drop_xy_does_not_track_grasp_xy_but_release_height_tracks_grasp():
    first_grasp_pose_6d = np.array([0.4, 0.0, 0.2, 0.0, np.pi / 2.0, 0.0])
    second_grasp_pose_6d = np.array([-0.7, 0.3, 0.1, 0.0, np.pi / 2.0, 0.0])

    first_plan = plan_safe_pick_place_steps(
        start_pose_6d=np.zeros(6),
        grasp_pose_6d=first_grasp_pose_6d,
        direct_to_grasp=True,
        execution_mode="safe_place",
    )
    second_plan = plan_safe_pick_place_steps(
        start_pose_6d=np.zeros(6),
        grasp_pose_6d=second_grasp_pose_6d,
        direct_to_grasp=True,
        execution_mode="safe_place",
    )

    np.testing.assert_allclose(first_plan.drop_pose_6d[:2], DROP_POSITION[:2])
    np.testing.assert_allclose(second_plan.drop_pose_6d[:2], DROP_POSITION[:2])
    assert first_plan.drop_pose_6d[2] == pytest.approx(first_grasp_pose_6d[2])
    assert second_plan.drop_pose_6d[2] == pytest.approx(second_grasp_pose_6d[2])


def test_drop_xyz_environment_names_configure_fixed_drop_position(monkeypatch):
    monkeypatch.setenv("DROP_X", "-0.12")
    monkeypatch.setenv("DROP_Y", "-0.34")
    monkeypatch.setenv("DROP_Z", "0.56")

    np.testing.assert_allclose(
        grasp_config_module.read_drop_position(),
        [-0.12, -0.34, 0.56],
    )


def test_rejects_zero_distance_pregrasp_when_approach_is_configured(monkeypatch):
    monkeypatch.setattr(
        trajectory_planner_module,
        "build_pre_grasp_pose_6d",
        lambda grasp_pose_6d: np.asarray(grasp_pose_6d, dtype=float).copy(),
    )
    with pytest.raises(ValueError, match="pre-grasp generation failed"):
        plan_safe_pick_place_steps(
            start_pose_6d=np.zeros(6),
            grasp_pose_6d=np.zeros(6),
        )


def test_transfer_to_drop_stays_at_or_above_lift_height_until_xy_motion_finishes():
    start_pose_6d = np.zeros(6)
    top_down_grasp_pose_6d = np.array(
        [0.4, 0.0, 0.2, np.pi, 0.0, 0.0]
    )

    plan = plan_safe_pick_place_steps(
        start_pose_6d=start_pose_6d,
        grasp_pose_6d=top_down_grasp_pose_6d,
        execution_mode="safe_place",
    )

    lift_step = plan.steps[5]
    hold_after_lift = plan.steps[6]
    transfer_step = plan.steps[7]
    descend_step = plan.steps[9]

    assert transfer_step.end_pose_6d[2] >= lift_step.end_pose_6d[2]
    assert descend_step.start_pose_6d[2] >= lift_step.end_pose_6d[2]
    assert descend_step.end_pose_6d[2] == pytest.approx(top_down_grasp_pose_6d[2])
    assert descend_step.end_pose_6d[2] < descend_step.start_pose_6d[2]
    assert hold_after_lift.name == "hold_after_lift"
    assert hold_after_lift.mode != "passive"
    assert hold_after_lift.duration_sec == pytest.approx(GRASP_LIFT_HOLD_SEC)
    assert hold_after_lift.duration_sec >= 5.0


@pytest.mark.parametrize("direct_to_grasp", [False, True])
def test_plan_holds_at_drop_pose_before_opening_for_release(direct_to_grasp):
    start_pose_6d = np.zeros(6)
    side_grasp_pose_6d = np.array([0.4, 0.0, 0.2, 0.0, np.pi / 2.0, 0.0])

    plan = plan_safe_pick_place_steps(
        start_pose_6d=start_pose_6d,
        grasp_pose_6d=side_grasp_pose_6d,
        direct_to_grasp=direct_to_grasp,
        execution_mode="safe_place",
    )

    step_names = [step.name for step in plan.steps]
    descend_index = step_names.index("descend_to_drop")
    hold_index = step_names.index("hold_before_release")
    open_index = step_names.index("open_gripper_to_release")
    hold_step = plan.steps[hold_index]

    assert hold_index == descend_index + 1
    assert open_index == hold_index + 1
    assert hold_step.action == "hold"
    assert hold_step.mode != "passive"
    assert hold_step.duration_sec == pytest.approx(RELEASE_PRE_OPEN_HOLD_SEC)
    assert hold_step.duration_sec == pytest.approx(1.0)
    np.testing.assert_allclose(hold_step.pose_6d, plan.drop_pose_6d)

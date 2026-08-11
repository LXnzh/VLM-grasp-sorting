import json
import os
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from my_course_pkg.grasp import config as grasp_config
from my_course_pkg.grasp import grasp_selector
from my_course_pkg.grasp import executor as executor_module
from my_course_pkg.grasp import trajectory_planner


NON_TUNA_ROUTING = {
    "tomato_soup_can": ("cylindrical_can", "side", 0.0),
    "pudding_box": ("box", "vertical", 0.0),
    "gelatin_box": ("box", "vertical", 0.0),
    "banana": ("banana", "centered", -0.02),
    "apple": ("round_top", "round_top", -0.02),
    "lemon": ("round_top", "round_top", -0.02),
    "peach": ("round_top", "round_top", -0.02),
    "pear": ("round_top", "round_top", 0.005),
    "orange": ("round_top", "round_top", -0.02),
    "plum": ("round_top", "round_top", -0.02),
    "sponge": ("box", "vertical", 0.0),
    "hammer": ("tool_top", "top_down", 0.0),
    "baseball": ("round_top", "round_top", -0.02),
    "tennis_ball": ("round_top", "round_top", -0.02),
    "racquetball": ("round_top", "round_top", -0.02),
    "foam_brick": ("box", "vertical", 0.0),
    "rubiks_cube": ("box", "vertical", 0.0),
}


def _step_signature(step):
    return (
        step.name,
        step.action,
        step.mode,
        step.gripper_position,
        step.duration_sec,
    )


def test_default_subprocess_routing_is_unchanged_without_tuna_environment():
    environment = {
        key: value
        for key, value in os.environ.items()
        if not key.startswith("GRASP_TUNA_")
    }
    script = """
import json
from my_course_pkg.grasp import config
print('ROUTING=' + json.dumps({
    name: [config.GRASP_CATEGORY_BY_OBJECT[name], config.GRASP_PROFILE_BY_OBJECT[name]]
    for name in sorted(config.YCB_GRASP_NAME_MAP)
    if name != 'tuna_fish_can'
}, sort_keys=True))
"""
    completed = subprocess.run(
        [sys.executable, "-c", script],
        check=True,
        capture_output=True,
        text=True,
        env=environment,
    )
    routing_line = next(
        line for line in completed.stdout.splitlines() if line.startswith("ROUTING=")
    )
    actual = json.loads(routing_line.removeprefix("ROUTING="))
    expected = {
        name: [category, profile]
        for name, (category, profile, _offset) in NON_TUNA_ROUTING.items()
    }
    assert actual == expected


def test_non_tuna_fixture_covers_every_configured_object_exactly_once():
    configured = set(grasp_config.YCB_GRASP_NAME_MAP) - {"tuna_fish_can"}
    assert set(NON_TUNA_ROUTING) == configured


@pytest.mark.parametrize("object_name", sorted(NON_TUNA_ROUTING))
def test_non_tuna_routing_and_candidate_matrices_remain_identical(
    monkeypatch,
    object_name,
):
    category, profile, z_offset = NON_TUNA_ROUTING[object_name]
    assert grasp_selector.get_grasp_category(object_name) == category
    assert grasp_selector.get_grasp_profile(object_name) == profile
    assert grasp_selector.get_grasp_z_offset(object_name) == pytest.approx(z_offset)

    T_world_object = np.eye(4, dtype=float)
    T_world_object[:3, 3] = [0.31, -0.27, 0.44]
    T_object_candidates = []
    for xyz in ([0.01, 0.02, 0.03], [-0.04, 0.05, 0.06]):
        candidate = np.eye(4, dtype=float)
        candidate[:3, 3] = xyz
        T_object_candidates.append(candidate)

    monkeypatch.setattr(
        grasp_selector,
        "get_selected_object_info",
        lambda _path: object_name,
    )
    monkeypatch.setattr(
        grasp_selector,
        "get_grasp_dir_for_object_name",
        lambda _name: "/semantic-baseline",
    )
    monkeypatch.setattr(
        grasp_selector,
        "load_grasps_for_object",
        lambda _path: object(),
    )
    monkeypatch.setattr(
        grasp_selector,
        "select_grasp_candidates",
        lambda *_args: [matrix.copy() for matrix in T_object_candidates],
    )
    monkeypatch.setattr(grasp_selector, "score_grasp", lambda *_args: 0.0)

    actual = grasp_selector.select_grasp_pose_candidates_6d(
        T_world_object.copy(),
        "/selected-object.json",
    )

    assert len(actual) == 2
    for index, (actual_pose, actual_matrix) in enumerate(actual):
        expected_matrix = T_world_object @ T_object_candidates[index]
        expected_matrix[2, 3] += z_offset
        np.testing.assert_allclose(
            actual_matrix,
            expected_matrix,
            rtol=0.0,
            atol=1e-12,
        )
        np.testing.assert_allclose(
            actual_pose[:3],
            expected_matrix[:3, 3],
            rtol=0.0,
            atol=1e-12,
        )
        np.testing.assert_allclose(
            actual_pose[3:],
            np.zeros(3),
            rtol=0.0,
            atol=1e-12,
        )


@pytest.mark.parametrize("object_name", sorted(NON_TUNA_ROUTING))
def test_non_tuna_plan_names_actions_targets_and_order_remain_identical(object_name):
    _category, profile, _z_offset = NON_TUNA_ROUTING[object_name]
    start_pose = np.array([0.1, -0.2, 0.6, np.pi, 0.0, 0.0], dtype=float)
    grasp_pose = np.array([0.42, -0.18, 0.31, np.pi, 0.0, 0.0], dtype=float)
    plan = trajectory_planner.plan_safe_pick_place_steps(
        start_pose_6d=start_pose,
        grasp_pose_6d=grasp_pose,
        direct_to_grasp=(profile == "side"),
        execution_mode="lift_return",
        debug_info={"object_name": object_name, "grasp_profile": profile},
    )

    expected_modes = [
        ("open_gripper_before_approach", "gripper", "", 0.0, 0.0),
        ("move_to_pre_grasp", "move", "moveit", None, 0.0),
        ("approach_grasp", "move", "cartesian", None, 0.0),
        ("close_gripper_at_grasp", "gripper", "", 0.79, 0.0),
        (
            "hold_after_close",
            "hold",
            "passive" if profile == "side" else "",
            None,
            3.0,
        ),
        ("lift_after_grasp", "move", "cartesian", None, 0.0),
        ("hold_after_lift", "hold", "", None, 5.0),
        ("return_to_grasp", "move", "cartesian", None, 0.0),
        ("hold_before_release", "hold", "", None, 1.0),
        ("open_gripper_to_release", "gripper", "", 0.0, 0.0),
        (
            "hold_after_release",
            "hold",
            "passive" if profile == "side" else "",
            None,
            2.0,
        ),
        ("retreat_after_release", "move", "cartesian", None, 0.0),
    ]
    assert [_step_signature(step) for step in plan.steps] == expected_modes

    expected_pregrasp = np.array(
        [0.42, -0.18, 0.41, np.pi, 0.0, 0.0],
        dtype=float,
    )
    expected_lift = np.array(
        [0.42, -0.18, 0.51, np.pi, 0.0, 0.0],
        dtype=float,
    )
    expected_return = np.array(
        [0.42, -0.18, 0.33, np.pi, 0.0, 0.0],
        dtype=float,
    )
    np.testing.assert_allclose(
        plan.pre_grasp_pose_6d,
        expected_pregrasp,
        rtol=0.0,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        plan.drop_high_pose_6d,
        expected_lift,
        rtol=0.0,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        plan.drop_pose_6d,
        expected_return,
        rtol=0.0,
        atol=1e-12,
    )
    assert plan.debug_info == {
        "object_name": object_name,
        "grasp_profile": profile,
    }


def test_explicit_regression_controls_cover_tomato_pudding_and_pear():
    assert NON_TUNA_ROUTING["tomato_soup_can"] == (
        "cylindrical_can",
        "side",
        0.0,
    )
    assert NON_TUNA_ROUTING["pudding_box"] == ("box", "vertical", 0.0)
    assert NON_TUNA_ROUTING["pear"] == ("round_top", "round_top", 0.005)


def test_non_tuna_executor_never_constructs_tuna_runtime(monkeypatch):
    monkeypatch.setenv("GRASP_TUNA_RADIAL_DIRECTION_COUNT", "invalid")

    def fail(*_args, **_kwargs):
        pytest.fail("non-Tuna execution must not construct Tuna coordinator")

    monkeypatch.setattr(executor_module, "TunaExecutionCoordinator", fail)
    executor = object.__new__(executor_module.ArmMotionExecutor)
    plan = SimpleNamespace(
        steps=[],
        debug_info={"object_name": "pear", "grasp_profile": "round_top"},
    )
    assert executor.execute_plan(plan) is None

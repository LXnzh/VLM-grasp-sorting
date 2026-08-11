from types import SimpleNamespace

import numpy as np
import pytest

from my_course_pkg.grasp import config as grasp_config
from my_course_pkg.grasp import pick_place_planner as planner_module
from my_course_pkg.grasp.config import APPROACH_DIST
from my_course_pkg.grasp.transforms import canonicalize_tabletop_object_pose


def make_transform(translation=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
    roll, pitch, yaw = rpy
    cx, sx = np.cos(roll), np.sin(roll)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cz, sz = np.cos(yaw), np.sin(yaw)
    rx = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, cx, -sx],
            [0.0, sx, cx],
        ]
    )
    ry = np.array(
        [
            [cy, 0.0, sy],
            [0.0, 1.0, 0.0],
            [-sy, 0.0, cy],
        ]
    )
    rz = np.array(
        [
            [cz, -sz, 0.0],
            [sz, cz, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    transform = np.eye(4)
    transform[:3, :3] = rz @ ry @ rx
    transform[:3, 3] = np.asarray(translation, dtype=float)
    return transform


def make_marker(
    name,
    center,
    scale=(0.04, 0.04, 0.08),
    marker_id=1,
    frame_id="world",
    orientation=(0.0, 0.0, 0.0, 1.0),
):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id),
        text=name,
        ns="scene_clearance_bounds",
        id=marker_id,
        pose=SimpleNamespace(
            position=SimpleNamespace(
                x=float(center[0]),
                y=float(center[1]),
                z=float(center[2]),
            ),
            orientation=SimpleNamespace(
                x=float(orientation[0]),
                y=float(orientation[1]),
                z=float(orientation[2]),
                w=float(orientation[3]),
            ),
        ),
        scale=SimpleNamespace(
            x=float(scale[0]),
            y=float(scale[1]),
            z=float(scale[2]),
        ),
    )


def make_clearance_obstacle(
    name,
    center,
    half_extents=(0.02, 0.02, 0.04),
    yaw_rad=0.0,
):
    half_extents = np.asarray(half_extents, dtype=float)
    T_world_bounds = make_transform(
        translation=center,
        rpy=(0.0, 0.0, yaw_rad),
    )
    return planner_module.SceneClearanceObstacle(
        name=name,
        center=np.asarray(center, dtype=float),
        radius_xy=float(np.hypot(half_extents[0], half_extents[1])),
        half_height=float(half_extents[2]),
        T_world_bounds=T_world_bounds,
        half_extents=half_extents,
    )


def make_clearance_config(
    profile="round_top",
    corridor_radius_m=0.03,
    clearance_margin_m=0.0,
    vertical_margin_m=0.01,
):
    return planner_module.ApproachClearanceConfig(
        profile=profile,
        enabled=True,
        require_scene=(profile == "round_top"),
        corridor_radius_m=corridor_radius_m,
        clearance_margin_m=clearance_margin_m,
        vertical_margin_m=vertical_margin_m,
    )


def make_planning_result(pregrasp_xyz, grasp_xyz, candidate_index=1):
    plan = SimpleNamespace(
        pre_grasp_pose_6d=np.array([*pregrasp_xyz, 0.0, 0.0, 0.0], dtype=float),
        grasp_pose_6d=np.array([*grasp_xyz, 0.0, 0.0, 0.0], dtype=float),
        steps=[],
        debug_info={},
    )
    return planner_module.PickPlacePlanningResult(
        plan=plan,
        T_world_obj=np.eye(4),
        T_world_grasp=np.eye(4),
        grasp_pose_6d=plan.grasp_pose_6d,
        candidate_index=candidate_index,
        candidate_count=1,
    )


@pytest.fixture(autouse=True)
def use_configured_safe_place_without_scene_for_unrelated_planner_tests(monkeypatch):
    monkeypatch.setattr(planner_module, "GRASP_PLACE_USE_SCENE", False)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_REQUIRE_SCENE", False)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_BASE_EXCLUSION_ENABLED", False)


def test_lift_return_nonvertical_planner_skips_scene_and_safe_place_search(
    monkeypatch,
):
    grasp_pose_6d = np.array([0.4, -0.2, 0.3, np.pi, 0.0, 0.0])
    T_world_grasp = np.eye(4)
    T_world_grasp[:3, 3] = grasp_pose_6d[:3]
    fake_plan = SimpleNamespace(
        pre_grasp_pose_6d=grasp_pose_6d + np.array([0.0, 0.0, 0.1, 0.0, 0.0, 0.0]),
        grasp_pose_6d=grasp_pose_6d,
        steps=[],
        debug_info={},
    )
    captured = {}

    monkeypatch.setattr(planner_module, "GRASP_EXECUTION_MODE", "lift_return", raising=False)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_ENABLED", True)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_USE_SCENE", True)
    monkeypatch.setattr(
        planner_module,
        "estimate_object_world_pose",
        lambda *args: np.eye(4),
    )
    monkeypatch.setattr(
        planner_module,
        "get_selected_object_info",
        lambda *args: "hammer",
    )
    monkeypatch.setattr(
        planner_module,
        "select_grasp_pose_candidates_6d",
        lambda *args: [(grasp_pose_6d, T_world_grasp)],
    )
    monkeypatch.setattr(
        planner_module,
        "_load_scene_clearance_obstacles",
        lambda *args: pytest.fail("lift_return must not load scene for placement"),
    )
    monkeypatch.setattr(
        planner_module,
        "_load_vertical_target_clearance_bounds",
        lambda *args: pytest.fail("non-vertical grasp must not load target bounds"),
    )
    monkeypatch.setattr(
        planner_module,
        "_select_safe_place_xy",
        lambda *args: pytest.fail("lift_return must not search for a safe place"),
    )

    def fake_plan_steps(**kwargs):
        captured.update(kwargs)
        return fake_plan

    monkeypatch.setattr(
        planner_module,
        "plan_safe_pick_place_steps",
        fake_plan_steps,
    )

    results = planner_module.plan_pick_place_candidates_from_perception(
        node=object(),
        start_pose_6d=np.zeros(6),
    )

    assert len(results) == 1
    assert captured["execution_mode"] == "lift_return"
    assert captured["drop_pose_6d"] is None
    assert captured["debug_info"]["grasp_profile"] == "top_down"
    assert "pear_minimum_close_position_rad" not in captured["debug_info"]


def test_pear_planner_carries_candidate_specific_close_readiness(monkeypatch):
    center = np.asarray(
        grasp_config.OBJECT_GEOMETRY_BY_NAME["pear"]["center"],
        dtype=float,
    )
    grasp_pose_6d = np.array(
        [center[0], center[1], center[2] + 0.02, np.pi, 0.0, 0.0],
        dtype=float,
    )
    T_world_grasp = make_transform(
        translation=grasp_pose_6d[:3],
        rpy=grasp_pose_6d[3:],
    )
    fake_plan = SimpleNamespace(
        pre_grasp_pose_6d=grasp_pose_6d
        + np.array([0.0, 0.0, 0.1, 0.0, 0.0, 0.0]),
        grasp_pose_6d=grasp_pose_6d,
        steps=[],
        debug_info={},
    )
    captured = {}

    monkeypatch.setattr(planner_module, "GRASP_EXECUTION_MODE", "lift_return")
    monkeypatch.setattr(
        planner_module,
        "estimate_object_world_pose",
        lambda *args: np.eye(4),
    )
    monkeypatch.setattr(
        planner_module,
        "get_selected_object_info",
        lambda *args: "pear",
    )
    monkeypatch.setattr(
        planner_module,
        "select_grasp_pose_candidates_6d",
        lambda *args: [(grasp_pose_6d, T_world_grasp)],
    )
    monkeypatch.setattr(
        planner_module,
        "_load_scene_clearance_obstacles",
        lambda *args: [],
    )

    def fake_plan_steps(**kwargs):
        captured.update(kwargs)
        return fake_plan

    monkeypatch.setattr(
        planner_module,
        "plan_safe_pick_place_steps",
        fake_plan_steps,
    )

    results = planner_module.plan_pick_place_candidates_from_perception(
        node=object(),
        start_pose_6d=np.zeros(6),
    )

    assert len(results) == 1
    debug = captured["debug_info"]
    assert debug["object_name"] == "pear"
    assert debug["pear_projected_width_m"] == pytest.approx(0.066546)
    assert debug["pear_expected_close_position_rad"] > 0.16
    assert debug["pear_minimum_close_position_rad"] > 0.15
    assert (
        debug["pear_minimum_close_position_rad"]
        < debug["pear_expected_close_position_rad"]
    )


def test_vertical_planner_centers_recorded_foam_candidate_and_pregrasp(
    monkeypatch,
    capsys,
):
    original_grasp_pose = np.array(
        [-0.5631, -0.5719, 0.8713, np.pi, 0.0, 0.0],
        dtype=float,
    )
    T_world_grasp = make_transform(
        translation=original_grasp_pose[:3],
        rpy=original_grasp_pose[3:],
    )
    target = make_clearance_obstacle(
        "foam_brick",
        center=(-0.4996, -0.9392, 0.9160),
        half_extents=(0.0266, 0.0392, 0.0260),
    )
    baseline_plan = planner_module.plan_safe_pick_place_steps(
        start_pose_6d=np.zeros(6),
        grasp_pose_6d=original_grasp_pose,
        drop_pose_6d=None,
        direct_to_grasp=False,
        execution_mode="lift_return",
        debug_info={},
    )

    monkeypatch.setattr(planner_module, "GRASP_EXECUTION_MODE", "lift_return")
    monkeypatch.setattr(
        planner_module,
        "estimate_object_world_pose",
        lambda *args: np.eye(4),
    )
    monkeypatch.setattr(
        planner_module,
        "get_selected_object_info",
        lambda *args: "foam_brick",
    )
    monkeypatch.setattr(
        planner_module,
        "select_grasp_pose_candidates_6d",
        lambda *args: [(original_grasp_pose, T_world_grasp)],
    )
    monkeypatch.setattr(
        planner_module,
        "_load_vertical_target_clearance_bounds",
        lambda *args: target,
    )

    result = planner_module.plan_pick_place_candidates_from_perception(
        node=object(),
        start_pose_6d=np.zeros(6),
    )[0]

    expected_grasp_xyz = np.array([-0.4996, -0.9392, 0.8980])
    correction_xyz = expected_grasp_xyz - original_grasp_pose[:3]
    np.testing.assert_allclose(correction_xyz, [0.0635, -0.3673, 0.0267])
    np.testing.assert_allclose(result.grasp_pose_6d[:3], expected_grasp_xyz)
    np.testing.assert_allclose(result.T_world_grasp[:3, 3], expected_grasp_xyz)
    np.testing.assert_allclose(result.grasp_pose_6d[3:], original_grasp_pose[3:])
    np.testing.assert_allclose(
        result.plan.pre_grasp_pose_6d[:3] - baseline_plan.pre_grasp_pose_6d[:3],
        correction_xyz,
    )
    np.testing.assert_allclose(
        result.plan.debug_info["vertical_grasp_correction_xy"],
        correction_xyz[:2],
    )
    np.testing.assert_allclose(
        result.plan.debug_info["vertical_grasp_correction_xyz"],
        correction_xyz,
    )
    assert result.plan.debug_info["vertical_bounds_bottom_z"] == pytest.approx(0.8900)
    assert result.plan.debug_info["vertical_minimum_tcp_z"] == pytest.approx(0.8980)
    steps = {step.name: step for step in result.plan.steps}
    np.testing.assert_allclose(
        steps["lift_after_grasp"].start_pose_6d[:3],
        expected_grasp_xyz,
    )
    assert steps["lift_after_grasp"].end_pose_6d[2] == pytest.approx(1.0980)
    assert steps["return_to_grasp"].end_pose_6d[2] == pytest.approx(0.9180)
    output = capsys.readouterr().out
    assert "Vertical grasp centering" in output
    assert "foam_brick" in output
    assert "bounds_bottom_z=0.8900" in output
    assert "minimum_tcp_z=0.8980" in output


def test_vertical_planner_fails_closed_when_target_bounds_are_unavailable(
    monkeypatch,
):
    grasp_pose = np.array([0.4, -0.2, 0.3, np.pi, 0.0, 0.0])
    T_world_grasp = make_transform(
        translation=grasp_pose[:3],
        rpy=grasp_pose[3:],
    )
    monkeypatch.setattr(
        planner_module,
        "estimate_object_world_pose",
        lambda *args: np.eye(4),
    )
    monkeypatch.setattr(
        planner_module,
        "get_selected_object_info",
        lambda *args: "foam_brick",
    )
    monkeypatch.setattr(
        planner_module,
        "select_grasp_pose_candidates_6d",
        lambda *args: [(grasp_pose, T_world_grasp)],
    )

    def fail_target_load(*args):
        raise RuntimeError("vertical target bounds unavailable for foam_brick")

    monkeypatch.setattr(
        planner_module,
        "_load_vertical_target_clearance_bounds",
        fail_target_load,
    )
    monkeypatch.setattr(
        planner_module,
        "plan_safe_pick_place_steps",
        lambda **kwargs: pytest.fail("motion plan must not be built without target bounds"),
    )

    with pytest.raises(RuntimeError, match="target bounds unavailable.*foam_brick"):
        planner_module.plan_pick_place_candidates_from_perception(
            node=object(),
            start_pose_6d=np.zeros(6),
        )


def test_tomato_can_pose_canonicalization_preserves_translation_and_z_upright():
    raw_pose = make_transform(
        translation=(0.4, -0.1, 0.52),
        rpy=(0.86, 0.78, 1.37),
    )

    canonical_pose = canonicalize_tabletop_object_pose(
        raw_pose,
        "tomato_soup_can",
    )

    np.testing.assert_allclose(canonical_pose[:3, 3], raw_pose[:3, 3])
    np.testing.assert_allclose(
        canonical_pose[:3, 2],
        [0.0, 0.0, 1.0],
        atol=1e-8,
    )


def test_side_profile_uses_real_pregrasp(monkeypatch):
    grasp_pose = np.array([0.4, 0.0, 0.2, 0.0, np.pi / 2.0, 0.0])
    T_world_grasp = np.eye(4)
    T_world_grasp[:3, 3] = grasp_pose[:3]

    monkeypatch.setattr(
        planner_module,
        "estimate_object_world_pose",
        lambda *args: np.eye(4),
    )
    monkeypatch.setattr(
        planner_module,
        "get_selected_object_info",
        lambda *args: "tomato_soup_can",
    )
    monkeypatch.setattr(planner_module, "get_grasp_profile", lambda *args: "side")
    monkeypatch.setattr(planner_module, "get_grasp_z_offset", lambda *args: 0.0)
    monkeypatch.setattr(
        planner_module,
        "select_grasp_pose_candidates_6d",
        lambda *args: [(grasp_pose, T_world_grasp)],
    )

    result = planner_module.plan_pick_place_candidates_from_perception(
        node=object(),
        start_pose_6d=np.zeros(6),
    )[0]

    plan = result.plan
    assert np.linalg.norm(
        plan.grasp_pose_6d[:3] - plan.pre_grasp_pose_6d[:3]
    ) == pytest.approx(APPROACH_DIST)
    assert any(step.name == "approach_grasp" for step in plan.steps)


def test_planner_uses_canonical_pose_for_tomato_can_grasp_selection(monkeypatch):
    raw_object_pose = make_transform(
        translation=(0.4, -0.1, 0.52),
        rpy=(0.86, 0.78, 1.37),
    )
    captured = {}

    grasp_pose = np.array([0.4, 0.0, 0.61, 0.0, np.pi / 2.0, 0.0])
    T_world_grasp = np.eye(4)
    T_world_grasp[:3, 3] = grasp_pose[:3]

    def fake_select_grasps(T_world_obj, *args):
        captured["T_world_obj"] = T_world_obj.copy()
        return [(grasp_pose, T_world_grasp)]

    monkeypatch.setattr(
        planner_module,
        "estimate_object_world_pose",
        lambda *args: raw_object_pose,
    )
    monkeypatch.setattr(
        planner_module,
        "get_selected_object_info",
        lambda *args: "tomato_soup_can",
    )
    monkeypatch.setattr(planner_module, "get_grasp_profile", lambda *args: "side")
    monkeypatch.setattr(planner_module, "get_grasp_z_offset", lambda *args: 0.0)
    monkeypatch.setattr(
        planner_module,
        "select_grasp_pose_candidates_6d",
        fake_select_grasps,
    )

    planner_module.plan_pick_place_candidates_from_perception(
        node=object(),
        start_pose_6d=np.zeros(6),
    )

    np.testing.assert_allclose(
        captured["T_world_obj"][:3, 3],
        raw_object_pose[:3, 3],
    )
    np.testing.assert_allclose(
        captured["T_world_obj"][:3, 2],
        [0.0, 0.0, 1.0],
        atol=1e-8,
    )


def test_planner_rejects_side_grasp_too_close_to_object_base(monkeypatch):
    object_pose = make_transform(translation=(0.4, -0.1, 0.52))
    low_grasp_pose = np.array([0.4, 0.0, 0.53, 0.0, np.pi / 2.0, 0.0])
    T_world_grasp = np.eye(4)
    T_world_grasp[:3, 3] = low_grasp_pose[:3]

    monkeypatch.setattr(
        planner_module,
        "estimate_object_world_pose",
        lambda *args: object_pose,
    )
    monkeypatch.setattr(
        planner_module,
        "get_selected_object_info",
        lambda *args: "tomato_soup_can",
    )
    monkeypatch.setattr(planner_module, "get_grasp_profile", lambda *args: "side")
    monkeypatch.setattr(planner_module, "get_grasp_z_offset", lambda *args: 0.0)
    monkeypatch.setattr(
        planner_module,
        "select_grasp_pose_candidates_6d",
        lambda *args: [(low_grasp_pose, T_world_grasp)],
    )

    with pytest.raises(RuntimeError, match="world-frame final/pregrasp z"):
        planner_module.plan_pick_place_candidates_from_perception(
            node=object(),
            start_pose_6d=np.zeros(6),
        )


def test_scene_clearance_obstacles_are_generic_and_ignore_target():
    marker_array = SimpleNamespace(
        markers=[
            make_marker("005_tomato_soup_can", (0.0, 0.0, 0.7), marker_id=1),
            make_marker("apple", (0.2, 0.0, 0.7), marker_id=2),
            make_marker("foam_brick", (0.3, 0.0, 0.7), marker_id=3),
        ]
    )

    obstacles = planner_module._scene_clearance_obstacles_from_marker_array(
        marker_array,
        "tomato soup can",
    )

    assert [obstacle.name for obstacle in obstacles] == ["apple", "foam_brick"]


def test_vertical_target_bounds_normalize_name_and_transform_into_world(monkeypatch):
    marker_array = SimpleNamespace(
        markers=[
            make_marker(
                "005_Foam-Brick",
                (0.2, 0.3, 0.4),
                scale=(0.06, 0.08, 0.05),
                frame_id="base_link",
            ),
            make_marker("apple", (0.0, 0.0, 0.7), marker_id=2),
        ]
    )
    T_world_base = make_transform(
        translation=(-0.7, -1.2, 0.5),
        rpy=(0.0, 0.0, np.pi / 2.0),
    )
    node = SimpleNamespace(tf_buffer=object())
    monkeypatch.setattr(
        planner_module,
        "get_transform_checked",
        lambda *args, **kwargs: T_world_base,
    )

    target = planner_module._target_clearance_bounds_from_marker_array(
        marker_array,
        "foam brick",
        node=node,
    )

    expected_center = (T_world_base @ np.array([0.2, 0.3, 0.4, 1.0]))[:3]
    assert target.name == "005_Foam-Brick"
    np.testing.assert_allclose(target.center, expected_center)
    np.testing.assert_allclose(target.half_extents, [0.03, 0.04, 0.025])


def test_vertical_target_bounds_require_exactly_one_matching_marker():
    with pytest.raises(RuntimeError, match="exactly one.*foam_brick.*found 0"):
        planner_module._target_clearance_bounds_from_marker_array(
            SimpleNamespace(markers=[make_marker("apple", (0.0, 0.0, 0.7))]),
            "foam_brick",
        )

    duplicate_markers = SimpleNamespace(
        markers=[
            make_marker("foam_brick", (0.1, 0.2, 0.3), marker_id=1),
            make_marker("005 foam brick", (0.1, 0.2, 0.3), marker_id=2),
        ]
    )
    with pytest.raises(RuntimeError, match="exactly one.*foam_brick.*found 2"):
        planner_module._target_clearance_bounds_from_marker_array(
            duplicate_markers,
            "foam_brick",
        )


def test_vertical_target_bounds_reject_invalid_matching_marker():
    marker_array = SimpleNamespace(
        markers=[
            make_marker(
                "foam_brick",
                (0.1, 0.2, 0.3),
                scale=(0.06, 0.0, 0.05),
            )
        ]
    )

    with pytest.raises(RuntimeError, match="usable target bounds.*foam_brick"):
        planner_module._target_clearance_bounds_from_marker_array(
            marker_array,
            "foam_brick",
        )


def test_vertical_target_bounds_require_world_transform_for_nonworld_marker():
    marker_array = SimpleNamespace(
        markers=[
            make_marker(
                "foam_brick",
                (0.1, 0.2, 0.3),
                frame_id="base_link",
            )
        ]
    )

    with pytest.raises(RuntimeError, match="requires TF conversion.*base_link.*world"):
        planner_module._target_clearance_bounds_from_marker_array(
            marker_array,
            "foam_brick",
        )


def test_vertical_target_bounds_loader_fails_closed_on_timeout(monkeypatch):
    monkeypatch.setattr(
        planner_module,
        "_wait_for_marker_array",
        lambda *args: None,
    )

    with pytest.raises(
        RuntimeError,
        match="requires target bounds.*foam_brick.*scene_clearance_bounds",
    ):
        planner_module._load_vertical_target_clearance_bounds(
            node=object(),
            selected_object_name="foam_brick",
        )


@pytest.mark.parametrize("object_name", ["foam_brick", "pudding_box"])
@pytest.mark.parametrize(
    "original_z, expected_z",
    [
        (0.8713, 0.8980),
        (0.9400, 0.9400),
        (0.8980, 0.8980),
    ],
)
def test_vertical_candidate_centering_applies_generic_bounds_floor_clamp(
    object_name,
    original_z,
    expected_z,
):
    grasp_pose = np.array([0.42, -0.18, original_z, np.pi, 0.1, -0.2])
    T_world_grasp = make_transform(
        translation=grasp_pose[:3],
        rpy=grasp_pose[3:],
    )
    target = make_clearance_obstacle(
        object_name,
        center=(-0.50, -0.94, 0.916),
        half_extents=(0.03, 0.04, 0.026),
    )
    original_pose = grasp_pose.copy()
    original_transform = T_world_grasp.copy()

    centered_pose, centered_transform, correction_xyz = (
        planner_module._center_vertical_grasp_candidate(
            grasp_pose,
            T_world_grasp,
            target,
        )
    )

    np.testing.assert_allclose(
        correction_xyz,
        [-0.92, -0.76, expected_z - original_z],
        atol=1e-12,
    )
    np.testing.assert_allclose(centered_pose[:2], target.center[:2])
    assert centered_pose[2] == pytest.approx(expected_z)
    np.testing.assert_allclose(centered_transform[:3, 3], centered_pose[:3])
    np.testing.assert_allclose(centered_pose[3:], original_pose[3:])
    np.testing.assert_allclose(centered_transform[:3, :3], original_transform[:3, :3])
    np.testing.assert_allclose(grasp_pose, original_pose)
    np.testing.assert_allclose(T_world_grasp, original_transform)


@pytest.mark.parametrize("invalid_half_height", [0.0, -0.01, np.nan, np.inf])
def test_vertical_candidate_centering_rejects_invalid_bounds_half_height(
    invalid_half_height,
):
    grasp_pose = np.array([0.42, -0.18, 0.8713, np.pi, 0.1, -0.2])
    T_world_grasp = make_transform(
        translation=grasp_pose[:3],
        rpy=grasp_pose[3:],
    )
    valid_target = make_clearance_obstacle(
        "foam_brick",
        center=(-0.50, -0.94, 0.916),
        half_extents=(0.03, 0.04, 0.026),
    )
    invalid_target = planner_module.SceneClearanceObstacle(
        name=valid_target.name,
        center=valid_target.center,
        radius_xy=valid_target.radius_xy,
        half_height=invalid_half_height,
        T_world_bounds=valid_target.T_world_bounds,
        half_extents=valid_target.half_extents,
    )

    with pytest.raises(RuntimeError, match="half-height"):
        planner_module._center_vertical_grasp_candidate(
            grasp_pose,
            T_world_grasp,
            invalid_target,
        )


def test_scene_clearance_loader_subscribes_only_to_bounds_topic():
    marker_array = SimpleNamespace(
        markers=[make_marker("apple", (0.0, 0.0, 0.7), marker_id=1)]
    )

    class FakeNode:
        cbg = object()

        def __init__(self):
            self.topics = []
            self.destroyed = []

        def create_subscription(
            self,
            message_type,
            topic,
            callback,
            qos,
            callback_group=None,
        ):
            self.topics.append(topic)
            callback(marker_array)
            return object()

        def destroy_subscription(self, subscription):
            self.destroyed.append(subscription)

    node = FakeNode()

    received = planner_module._wait_for_marker_array(
        node,
        grasp_config.GRASP_SCENE_CLEARANCE_TOPIC,
        0.01,
    )

    assert received is marker_array
    assert node.topics == ["/scene_clearance_bounds"]
    assert len(node.destroyed) == 1


def test_scene_clearance_radius_covers_aabb_corners():
    marker_array = SimpleNamespace(
        markers=[
            make_marker(
                "hammer",
                (0.0, 0.0, 0.7),
                scale=(0.06, 0.08, 0.10),
            )
        ]
    )

    obstacles = planner_module._scene_clearance_obstacles_from_marker_array(
        marker_array,
        "apple",
    )

    assert obstacles[0].radius_xy == pytest.approx(0.05)
    assert obstacles[0].half_height == pytest.approx(0.05)
    np.testing.assert_allclose(obstacles[0].half_extents, [0.03, 0.04, 0.05])
    np.testing.assert_allclose(
        obstacles[0].T_world_bounds,
        make_transform(translation=(0.0, 0.0, 0.7)),
    )
    np.testing.assert_allclose(obstacles[0].center, [0.0, 0.0, 0.7])
    assert obstacles[0].T_world_bounds[2, 3] == pytest.approx(0.7)


@pytest.mark.parametrize(
    "scale",
    [
        (0.0, 0.1, 0.1),
        (-0.1, 0.1, 0.1),
        (np.nan, 0.1, 0.1),
        (np.inf, 0.1, 0.1),
        (0.1, 0.1, 0.0),
    ],
)
def test_scene_clearance_rejects_invalid_scale(scale):
    marker_array = SimpleNamespace(
        markers=[make_marker("hammer", (0.0, 0.0, 0.7), scale=scale)]
    )

    with pytest.raises(RuntimeError, match="Could not build clearance obstacle"):
        planner_module._scene_clearance_obstacles_from_marker_array(
            marker_array,
            "apple",
        )


def test_scene_clearance_rechecks_center_after_tf_transform(monkeypatch):
    marker_array = SimpleNamespace(
        markers=[
            make_marker(
                "hammer",
                (0.0, 0.0, 0.7),
                frame_id="base_link",
            )
        ]
    )
    invalid_transform = np.eye(4)
    invalid_transform[0, 3] = np.nan
    monkeypatch.setattr(
        planner_module,
        "get_transform_checked",
        lambda *args, **kwargs: invalid_transform,
    )

    with pytest.raises(RuntimeError, match="Could not build clearance obstacle"):
        planner_module._scene_clearance_obstacles_from_marker_array(
            marker_array,
            "apple",
            node=SimpleNamespace(tf_buffer=object()),
        )


def test_scene_clearance_rejects_non_rigid_tf(monkeypatch):
    marker_array = SimpleNamespace(
        markers=[make_marker("hammer", (0.0, 0.0, 0.7), frame_id="base_link")]
    )
    invalid_transform = np.eye(4)
    invalid_transform[0, 0] = 2.0
    monkeypatch.setattr(
        planner_module,
        "get_transform_checked",
        lambda *args, **kwargs: invalid_transform,
    )

    with pytest.raises(RuntimeError, match="Could not build clearance obstacle"):
        planner_module._scene_clearance_obstacles_from_marker_array(
            marker_array,
            "apple",
            node=SimpleNamespace(tf_buffer=object()),
        )


def test_scene_clearance_rejects_tf_with_roll_or_pitch(monkeypatch):
    marker_array = SimpleNamespace(
        markers=[make_marker("hammer", (0.0, 0.0, 0.7), frame_id="base_link")]
    )
    monkeypatch.setattr(
        planner_module,
        "get_transform_checked",
        lambda *args, **kwargs: make_transform(rpy=(0.1, 0.0, 0.0)),
    )

    with pytest.raises(RuntimeError, match="Could not build clearance obstacle"):
        planner_module._scene_clearance_obstacles_from_marker_array(
            marker_array,
            "apple",
            node=SimpleNamespace(tf_buffer=object()),
        )


@pytest.mark.parametrize(
    "orientation",
    [
        (0.0, 0.0, 0.0, 0.0),
        (np.nan, 0.0, 0.0, 1.0),
        (0.0, 0.0, np.inf, 1.0),
    ],
)
def test_scene_clearance_rejects_invalid_marker_quaternion(orientation):
    marker_array = SimpleNamespace(
        markers=[
            make_marker(
                "hammer",
                (0.0, 0.0, 0.7),
                orientation=orientation,
            )
        ]
    )

    with pytest.raises(RuntimeError, match="Could not build clearance obstacle"):
        planner_module._scene_clearance_obstacles_from_marker_array(
            marker_array,
            "apple",
        )


def test_scene_clearance_target_only_array_returns_loaded_empty_list():
    marker_array = SimpleNamespace(
        markers=[make_marker("013_apple", (0.0, 0.0, 0.7))]
    )

    obstacles = planner_module._scene_clearance_obstacles_from_marker_array(
        marker_array,
        "apple",
    )

    assert obstacles == []


def test_scene_clearance_transforms_marker_centers_into_world(monkeypatch):
    marker_array = SimpleNamespace(
        markers=[
            make_marker(
                "apple",
                (0.2, 0.0, 0.7),
                marker_id=2,
                frame_id="base_link",
            ),
        ]
    )
    T_world_base = np.eye(4)
    T_world_base[:3, 3] = [1.0, 2.0, 0.0]

    monkeypatch.setattr(
        planner_module,
        "get_transform_checked",
        lambda *args, **kwargs: T_world_base,
    )

    obstacles = planner_module._scene_clearance_obstacles_from_marker_array(
        marker_array,
        "tomato soup can",
        node=SimpleNamespace(tf_buffer=object()),
    )

    np.testing.assert_allclose(obstacles[0].center, [1.2, 2.0, 0.7])
    np.testing.assert_allclose(
        obstacles[0].T_world_bounds[:3, 3],
        [1.2, 2.0, 0.7],
    )


def test_scene_clearance_composes_marker_and_frame_yaw(monkeypatch):
    half_angle = np.pi / 8.0
    marker_array = SimpleNamespace(
        markers=[
            make_marker(
                "hammer",
                (0.2, 0.0, 0.7),
                scale=(0.06, 0.08, 0.10),
                frame_id="base_link",
                orientation=(0.0, 0.0, np.sin(half_angle), np.cos(half_angle)),
            )
        ]
    )
    T_world_base = make_transform(
        translation=(1.0, 2.0, 0.0),
        rpy=(0.0, 0.0, np.pi / 2.0),
    )
    monkeypatch.setattr(
        planner_module,
        "get_transform_checked",
        lambda *args, **kwargs: T_world_base,
    )

    obstacle = planner_module._scene_clearance_obstacles_from_marker_array(
        marker_array,
        "apple",
        node=SimpleNamespace(tf_buffer=object()),
    )[0]

    expected = T_world_base @ make_transform(
        translation=(0.2, 0.0, 0.7),
        rpy=(0.0, 0.0, np.pi / 4.0),
    )
    np.testing.assert_allclose(obstacle.T_world_bounds, expected, atol=1e-9)
    np.testing.assert_allclose(obstacle.half_extents, [0.03, 0.04, 0.05])


def test_scene_clearance_rejects_unusable_non_target_marker():
    marker_array = SimpleNamespace(
        markers=[
            make_marker("apple", (np.nan, 0.0, 0.7), marker_id=2),
        ]
    )

    with pytest.raises(RuntimeError, match="Could not build clearance obstacle"):
        planner_module._scene_clearance_obstacles_from_marker_array(
            marker_array,
            "tomato soup can",
        )


def test_safe_place_selection_avoids_all_non_target_obstacles(monkeypatch):
    monkeypatch.setattr(planner_module, "GRASP_PLACE_X_MIN", 0.0)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_X_MAX", 0.3)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_Y_MIN", 0.0)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_Y_MAX", 0.0)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_EDGE_MARGIN_M", 0.0)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_GRID_STEP_M", 0.1)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_OBJECT_CLEARANCE_M", 0.08)
    obstacle = planner_module.SceneClearanceObstacle(
        name="banana",
        center=np.array([0.0, 0.0, 0.7], dtype=float),
        radius_xy=0.05,
        half_height=0.04,
    )

    selection = planner_module._select_safe_place_xy(
        [obstacle],
        preferred_xy=np.array([0.0, 0.0]),
    )

    assert selection.xy[0] >= 0.2
    assert selection.min_clearance_m >= 0.08


def test_base_exclusion_zone_rejects_clear_candidate(monkeypatch):
    monkeypatch.setattr(planner_module, "GRASP_PLACE_BASE_EXCLUSION_ENABLED", True)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_BASE_EXCLUSION_X_MIN", -0.05)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_BASE_EXCLUSION_X_MAX", 0.05)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_BASE_EXCLUSION_Y_MIN", -0.05)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_BASE_EXCLUSION_Y_MAX", 0.05)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_X_MIN", 0.0)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_X_MAX", 0.1)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_Y_MIN", 0.0)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_Y_MAX", 0.0)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_EDGE_MARGIN_M", 0.0)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_GRID_STEP_M", 0.1)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_OBJECT_CLEARANCE_M", 0.08)

    selection = planner_module._select_safe_place_xy(
        [],
        preferred_xy=np.array([0.0, 0.0]),
    )

    np.testing.assert_allclose(selection.xy, [0.1, 0.0])
    assert not planner_module._in_base_exclusion_zone(
        selection.xy,
        -0.05,
        0.05,
        -0.05,
        0.05,
    )


def test_safe_place_selection_reports_when_base_blocks_all_points(monkeypatch):
    monkeypatch.setattr(planner_module, "GRASP_PLACE_BASE_EXCLUSION_ENABLED", True)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_BASE_EXCLUSION_X_MIN", -0.05)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_BASE_EXCLUSION_X_MAX", 0.05)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_BASE_EXCLUSION_Y_MIN", -0.05)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_BASE_EXCLUSION_Y_MAX", 0.05)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_X_MIN", 0.0)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_X_MAX", 0.0)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_Y_MIN", 0.0)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_Y_MAX", 0.0)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_EDGE_MARGIN_M", 0.0)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_GRID_STEP_M", 0.1)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_OBJECT_CLEARANCE_M", 0.08)

    with pytest.raises(RuntimeError, match="rejected_by_base=1"):
        planner_module._select_safe_place_xy(
            [],
            preferred_xy=np.array([0.0, 0.0]),
        )


def test_mid_table_policy_selects_near_preferred_when_clearance_ties(monkeypatch):
    monkeypatch.setattr(planner_module, "GRASP_PLACE_BASE_EXCLUSION_ENABLED", True)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_BASE_EXCLUSION_X_MIN", -0.05)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_BASE_EXCLUSION_X_MAX", 0.35)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_BASE_EXCLUSION_Y_MIN", -0.45)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_BASE_EXCLUSION_Y_MAX", 0.10)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_X_MIN", -0.55)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_X_MAX", -0.20)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_Y_MIN", -0.85)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_Y_MAX", -0.50)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_EDGE_MARGIN_M", 0.0)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_GRID_STEP_M", 0.05)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_OBJECT_CLEARANCE_M", 0.08)
    preferred = np.array([-0.38, -0.65], dtype=float)

    selection = planner_module._select_safe_place_xy(
        [],
        preferred_xy=preferred,
    )

    assert -0.55 <= selection.xy[0] <= -0.20
    assert -0.85 <= selection.xy[1] <= -0.50 + 1e-9
    assert np.linalg.norm(selection.xy - preferred) <= 0.05


def test_mid_table_policy_finds_valid_point_with_corner_obstacles(monkeypatch):
    monkeypatch.setattr(planner_module, "GRASP_PLACE_BASE_EXCLUSION_ENABLED", True)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_BASE_EXCLUSION_X_MIN", -0.05)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_BASE_EXCLUSION_X_MAX", 0.35)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_BASE_EXCLUSION_Y_MIN", -0.45)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_BASE_EXCLUSION_Y_MAX", 0.10)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_X_MIN", -0.55)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_X_MAX", -0.20)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_Y_MIN", -0.85)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_Y_MAX", -0.50)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_EDGE_MARGIN_M", 0.0)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_GRID_STEP_M", 0.05)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_OBJECT_CLEARANCE_M", 0.08)
    preferred = np.array([-0.38, -0.65], dtype=float)
    obstacles = [
        planner_module.SceneClearanceObstacle(
            name="foam_brick",
            center=np.array([-0.55, -0.85, 0.7], dtype=float),
            radius_xy=0.02,
            half_height=0.04,
        ),
        planner_module.SceneClearanceObstacle(
            name="cracker_box",
            center=np.array([-0.20, -0.50, 0.7], dtype=float),
            radius_xy=0.02,
            half_height=0.04,
        ),
    ]

    selection = planner_module._select_safe_place_xy(
        obstacles,
        preferred_xy=preferred,
    )

    assert -0.55 <= selection.xy[0] <= -0.20
    assert -0.85 <= selection.xy[1] <= -0.50 + 1e-9
    assert selection.min_clearance_m >= 0.08
    assert not planner_module._in_base_exclusion_zone(
        selection.xy,
        -0.05,
        0.35,
        -0.45,
        0.10,
    )


def test_safe_place_selection_fails_when_every_grid_point_is_blocked(monkeypatch):
    monkeypatch.setattr(planner_module, "GRASP_PLACE_X_MIN", 0.0)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_X_MAX", 0.0)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_Y_MIN", 0.0)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_Y_MAX", 0.0)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_EDGE_MARGIN_M", 0.0)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_GRID_STEP_M", 0.1)
    monkeypatch.setattr(planner_module, "GRASP_PLACE_OBJECT_CLEARANCE_M", 0.08)
    obstacle = planner_module.SceneClearanceObstacle(
        name="banana",
        center=np.array([0.0, 0.0, 0.7], dtype=float),
        radius_xy=0.05,
        half_height=0.04,
    )

    with pytest.raises(RuntimeError, match="No safe placement point"):
        planner_module._select_safe_place_xy(
            [obstacle],
            preferred_xy=np.array([0.0, 0.0]),
        )


def test_safe_drop_pose_uses_safe_xy_and_grasp_release_height():
    selection = planner_module.SafePlaceSelection(
        xy=np.array([-0.55, -0.45], dtype=float),
        min_clearance_m=0.2,
        obstacle_count=3,
        score=0.1,
    )
    grasp_pose = np.array([0.1, -0.9, 0.97, 0.0, np.pi / 2.0, 0.0])

    drop_pose = planner_module._build_safe_drop_pose_for_grasp(
        grasp_pose,
        selection,
    )

    np.testing.assert_allclose(drop_pose[:2], [-0.55, -0.45])
    assert drop_pose[2] == pytest.approx(grasp_pose[2])
    np.testing.assert_allclose(drop_pose[3:], grasp_pose[3:])


@pytest.mark.parametrize(
    ("point", "expected"),
    [
        ((0.0, 0.0), 0.0),
        ((0.15, 0.0), 0.05),
        ((0.15, 0.25), np.hypot(0.05, 0.05)),
    ],
)
def test_xy_point_to_rectangle_distance(point, expected):
    distance = planner_module._xy_point_to_rectangle_distance(
        point,
        half_extents_xy=(0.10, 0.20),
    )

    assert distance == pytest.approx(expected)


@pytest.mark.parametrize(
    ("segment_start", "segment_end", "expected"),
    [
        ((-0.2, 0.0), (0.2, 0.0), 0.0),
        ((-0.2, 0.15), (0.2, 0.15), 0.05),
        ((0.15, 0.15), (0.15, 0.15), np.hypot(0.05, 0.05)),
    ],
)
def test_xy_segment_to_rectangle_distance(
    segment_start,
    segment_end,
    expected,
):
    distance = planner_module._xy_segment_to_rectangle_distance(
        segment_start,
        segment_end,
        half_extents_xy=(0.10, 0.10),
    )

    assert distance == pytest.approx(expected)


def test_clip_segment_to_z_slab_handles_crossing_parallel_and_disjoint():
    assert planner_module._clip_segment_to_z_slab(
        (0.0, 0.0, 1.0),
        (0.0, 0.0, 0.0),
        z_min=0.4,
        z_max=0.6,
    ) == pytest.approx((0.4, 0.6))
    assert planner_module._clip_segment_to_z_slab(
        (0.0, 0.0, 0.5),
        (1.0, 0.0, 0.5),
        z_min=0.4,
        z_max=0.6,
    ) == pytest.approx((0.0, 1.0))
    assert (
        planner_module._clip_segment_to_z_slab(
            (0.0, 0.0, 0.7),
            (1.0, 0.0, 0.7),
            z_min=0.4,
            z_max=0.6,
        )
        is None
    )


def test_exact_corridor_uses_only_xy_segment_part_inside_z_slab():
    result = make_planning_result(
        pregrasp_xyz=(0.0, 0.0, 1.0),
        grasp_xyz=(1.0, 0.0, 0.0),
    )
    obstacle = make_clearance_obstacle(
        "banana",
        center=(0.9, 0.0, 0.9),
        half_extents=(0.02, 0.02, 0.02),
    )

    violation = planner_module._approach_clearance_violation(
        result.plan,
        [obstacle],
        make_clearance_config(vertical_margin_m=0.01),
    )

    assert violation is None


def test_exact_corridor_removes_circumscribed_circle_false_rejection():
    result = make_planning_result(
        pregrasp_xyz=(0.0, 0.0, 0.9),
        grasp_xyz=(0.0, 0.0, 0.7),
    )
    obstacle = make_clearance_obstacle(
        "banana",
        center=(0.1755, 0.0, 0.75),
        half_extents=(0.0548, 0.0894, 0.0186),
    )
    config = make_clearance_config(
        corridor_radius_m=0.08,
        clearance_margin_m=0.03,
        vertical_margin_m=0.12,
    )

    assert 0.1755 <= obstacle.radius_xy + 0.11
    assert planner_module._approach_clearance_violation(
        result.plan,
        [obstacle],
        config,
    ) is None
    assert planner_module._xy_segment_to_rectangle_distance(
        (-0.1755, 0.0),
        (-0.1755, 0.0),
        obstacle.half_extents[:2],
    ) == pytest.approx(0.1207)


def test_exact_corridor_rejects_true_face_clearance_and_reports_box_distance():
    result = make_planning_result(
        pregrasp_xyz=(0.0, 0.0, 0.9),
        grasp_xyz=(0.0, 0.0, 0.7),
    )
    obstacle = make_clearance_obstacle(
        "banana",
        center=(0.15, 0.0, 0.75),
        half_extents=(0.05, 0.08, 0.02),
    )

    violation = planner_module._approach_clearance_violation(
        result.plan,
        [obstacle],
        make_clearance_config(
            corridor_radius_m=0.08,
            clearance_margin_m=0.03,
            vertical_margin_m=0.12,
        ),
    )

    assert violation is not None
    assert violation.box_xy_distance_m == pytest.approx(0.10)
    assert violation.required_clearance_m == pytest.approx(0.11)
    assert violation.clearance_m == pytest.approx(-0.01)


def test_exact_corridor_rejects_true_corner_clearance():
    result = make_planning_result(
        pregrasp_xyz=(0.0, 0.0, 0.9),
        grasp_xyz=(0.0, 0.0, 0.7),
    )
    obstacle = make_clearance_obstacle(
        "foam_brick",
        center=(0.09, 0.09, 0.75),
        half_extents=(0.02, 0.02, 0.02),
    )
    violation = planner_module._approach_clearance_violation(
        result.plan,
        [obstacle],
        make_clearance_config(corridor_radius_m=0.10),
    )

    assert violation is not None
    assert violation.box_xy_distance_m == pytest.approx(np.hypot(0.07, 0.07))


@pytest.mark.parametrize(
    ("extra_clearance_m", "expect_violation"),
    [
        (0.0, True),
        (0.5e-9, True),
        (2.0e-9, False),
    ],
)
def test_exact_corridor_fixed_numeric_epsilon_is_safe_sided(
    extra_clearance_m,
    expect_violation,
):
    result = make_planning_result(
        pregrasp_xyz=(0.0, 0.0, 0.9),
        grasp_xyz=(0.0, 0.0, 0.7),
    )
    obstacle = make_clearance_obstacle(
        "banana",
        center=(0.13 + extra_clearance_m, 0.0, 0.75),
        half_extents=(0.02, 0.02, 0.02),
    )
    violation = planner_module._approach_clearance_violation(
        result.plan,
        [obstacle],
        make_clearance_config(
            corridor_radius_m=0.08,
            clearance_margin_m=0.03,
            vertical_margin_m=0.12,
        ),
    )

    assert (violation is not None) is expect_violation


def test_exact_corridor_honors_rotated_rectangle_direction():
    result = make_planning_result(
        pregrasp_xyz=(0.08, 0.0, 0.9),
        grasp_xyz=(0.08, 0.0, 0.7),
    )
    unrotated = make_clearance_obstacle(
        "banana",
        center=(0.0, 0.0, 0.75),
        half_extents=(0.10, 0.01, 0.02),
    )
    rotated = make_clearance_obstacle(
        "banana",
        center=(0.0, 0.0, 0.75),
        half_extents=(0.10, 0.01, 0.02),
        yaw_rad=np.pi / 2.0,
    )
    config = make_clearance_config(corridor_radius_m=0.03)

    assert planner_module._approach_clearance_violation(
        result.plan,
        [unrotated],
        config,
    ) is not None
    assert planner_module._approach_clearance_violation(
        result.plan,
        [rotated],
        config,
    ) is None


def test_exact_corridor_rejects_missing_precise_bounds_geometry():
    result = make_planning_result(
        pregrasp_xyz=(0.0, 0.0, 0.9),
        grasp_xyz=(0.0, 0.0, 0.7),
    )
    legacy_obstacle = planner_module.SceneClearanceObstacle(
        name="banana",
        center=np.array([0.2, 0.0, 0.75], dtype=float),
        radius_xy=0.05,
        half_height=0.02,
    )

    with pytest.raises(RuntimeError, match="precise bounds geometry"):
        planner_module._approach_clearance_violation(
            result.plan,
            [legacy_obstacle],
            make_clearance_config(),
        )


def test_side_approach_clearance_rejects_any_near_non_target(monkeypatch):
    monkeypatch.setattr(
        planner_module,
        "SIDE_GRASP_APPROACH_CORRIDOR_RADIUS_M",
        0.03,
    )
    monkeypatch.setattr(
        planner_module,
        "SIDE_GRASP_APPROACH_CLEARANCE_MARGIN_M",
        0.0,
    )
    monkeypatch.setattr(
        planner_module,
        "SIDE_GRASP_APPROACH_VERTICAL_MARGIN_M",
        0.01,
    )
    result = make_planning_result(
        pregrasp_xyz=(0.0, 0.0, 0.7),
        grasp_xyz=(0.1, 0.0, 0.7),
    )
    obstacle = make_clearance_obstacle(
        "apple",
        center=(0.05, 0.02, 0.7),
        half_extents=(0.02, 0.02, 0.04),
    )

    with pytest.raises(RuntimeError, match="No side-grasp candidate clears"):
        planner_module._filter_results_by_approach_clearance(
            [result],
            [obstacle],
            "tomato_soup_can",
        )


def test_side_approach_clearance_keeps_far_non_target(monkeypatch):
    monkeypatch.setattr(
        planner_module,
        "SIDE_GRASP_APPROACH_CORRIDOR_RADIUS_M",
        0.03,
    )
    monkeypatch.setattr(
        planner_module,
        "SIDE_GRASP_APPROACH_CLEARANCE_MARGIN_M",
        0.0,
    )
    result = make_planning_result(
        pregrasp_xyz=(0.0, 0.0, 0.7),
        grasp_xyz=(0.1, 0.0, 0.7),
    )
    obstacle = make_clearance_obstacle(
        "foam_brick",
        center=(0.05, 0.2, 0.7),
        half_extents=(0.02, 0.02, 0.04),
    )

    kept = planner_module._filter_results_by_approach_clearance(
        [result],
        [obstacle],
        "tomato_soup_can",
    )

    assert kept == [result]


def _set_round_top_clearance_test_config(monkeypatch):
    monkeypatch.setattr(
        planner_module,
        "ROUND_TOP_CLEARANCE_ENABLED",
        True,
        raising=False,
    )
    monkeypatch.setattr(
        planner_module,
        "ROUND_TOP_APPROACH_CORRIDOR_RADIUS_M",
        0.03,
        raising=False,
    )
    monkeypatch.setattr(
        planner_module,
        "ROUND_TOP_APPROACH_CLEARANCE_MARGIN_M",
        0.0,
        raising=False,
    )
    monkeypatch.setattr(
        planner_module,
        "ROUND_TOP_APPROACH_VERTICAL_MARGIN_M",
        0.01,
        raising=False,
    )
    monkeypatch.setattr(
        planner_module,
        "ROUND_TOP_CLEARANCE_REQUIRE_SCENE",
        True,
        raising=False,
    )


def test_round_top_vertical_margin_covers_lowest_finger_envelope():
    assert grasp_config.ROUND_TOP_APPROACH_VERTICAL_MARGIN_M >= abs(
        grasp_config.ROUND_TOP_TCP_TO_LOWEST_FINGER_Z_M
    )


def test_round_top_approach_clearance_rejects_near_non_target(monkeypatch):
    _set_round_top_clearance_test_config(monkeypatch)
    result = make_planning_result(
        pregrasp_xyz=(0.0, 0.0, 0.8),
        grasp_xyz=(0.0, 0.0, 0.7),
    )
    obstacle = make_clearance_obstacle(
        "banana",
        center=(0.04, 0.0, 0.75),
        half_extents=(0.02, 0.02, 0.04),
    )

    with pytest.raises(RuntimeError, match="No round_top grasp candidate clears"):
        planner_module._filter_results_by_approach_clearance(
            [result],
            [obstacle],
            "apple",
        )


def test_round_top_approach_clearance_keeps_far_non_target(monkeypatch):
    _set_round_top_clearance_test_config(monkeypatch)
    result = make_planning_result(
        pregrasp_xyz=(0.0, 0.0, 0.8),
        grasp_xyz=(0.0, 0.0, 0.7),
    )
    obstacle = make_clearance_obstacle(
        "hammer",
        center=(0.20, 0.0, 0.75),
        half_extents=(0.02, 0.02, 0.04),
    )

    kept = planner_module._filter_results_by_approach_clearance(
        [result],
        [obstacle],
        "apple",
    )

    assert kept == [result]


def test_round_top_approach_clearance_fails_closed_without_scene(monkeypatch):
    _set_round_top_clearance_test_config(monkeypatch)
    result = make_planning_result(
        pregrasp_xyz=(0.0, 0.0, 0.8),
        grasp_xyz=(0.0, 0.0, 0.7),
    )

    with pytest.raises(RuntimeError, match="round_top approach clearance requires"):
        planner_module._filter_results_by_approach_clearance(
            [result],
            None,
            "apple",
        )


def test_round_top_approach_clearance_accepts_loaded_empty_scene(monkeypatch):
    _set_round_top_clearance_test_config(monkeypatch)
    result = make_planning_result(
        pregrasp_xyz=(0.0, 0.0, 0.8),
        grasp_xyz=(0.0, 0.0, 0.7),
    )

    kept = planner_module._filter_results_by_approach_clearance(
        [result],
        [],
        "apple",
    )

    assert kept == [result]


def test_planner_raises_when_all_side_candidates_blocked_by_scene_object(
    monkeypatch,
):
    fake_plan = SimpleNamespace(
        pre_grasp_pose_6d=np.array([0.0, 0.0, 0.7, 0.0, 0.0, 0.0]),
        grasp_pose_6d=np.array([0.1, 0.0, 0.7, 0.0, 0.0, 0.0]),
        steps=[],
        debug_info={},
    )
    T_world_grasp = np.eye(4)
    T_world_grasp[:3, 3] = fake_plan.grasp_pose_6d[:3]
    obstacle = make_clearance_obstacle(
        "apple",
        center=(0.05, 0.0, 0.7),
        half_extents=(0.02, 0.02, 0.04),
    )

    monkeypatch.setattr(
        planner_module,
        "estimate_object_world_pose",
        lambda *args: np.eye(4),
    )
    monkeypatch.setattr(
        planner_module,
        "get_selected_object_info",
        lambda *args: "tomato_soup_can",
    )
    monkeypatch.setattr(planner_module, "get_grasp_profile", lambda *args: "side")
    monkeypatch.setattr(planner_module, "get_grasp_z_offset", lambda *args: 0.0)
    monkeypatch.setattr(
        planner_module,
        "select_grasp_pose_candidates_6d",
        lambda *args: [(fake_plan.grasp_pose_6d, T_world_grasp)],
    )
    monkeypatch.setattr(
        planner_module,
        "plan_safe_pick_place_steps",
        lambda **kwargs: fake_plan,
    )
    monkeypatch.setattr(
        planner_module,
        "_load_scene_clearance_obstacles",
        lambda *args: [obstacle],
    )

    with pytest.raises(RuntimeError, match="No side-grasp candidate clears"):
        planner_module.plan_pick_place_candidates_from_perception(
            node=object(),
            start_pose_6d=np.zeros(6),
        )


def test_planner_skips_side_clearance_when_scene_clearance_bounds_missing(
    monkeypatch,
):
    fake_plan = SimpleNamespace(
        pre_grasp_pose_6d=np.array([0.0, 0.0, 0.7, 0.0, 0.0, 0.0]),
        grasp_pose_6d=np.array([0.1, 0.0, 0.7, 0.0, 0.0, 0.0]),
        steps=[],
        debug_info={},
    )
    T_world_grasp = np.eye(4)
    T_world_grasp[:3, 3] = fake_plan.grasp_pose_6d[:3]

    monkeypatch.setattr(
        planner_module,
        "estimate_object_world_pose",
        lambda *args: np.eye(4),
    )
    monkeypatch.setattr(
        planner_module,
        "get_selected_object_info",
        lambda *args: "tomato_soup_can",
    )
    monkeypatch.setattr(planner_module, "get_grasp_profile", lambda *args: "side")
    monkeypatch.setattr(planner_module, "get_grasp_z_offset", lambda *args: 0.0)
    monkeypatch.setattr(
        planner_module,
        "select_grasp_pose_candidates_6d",
        lambda *args: [(fake_plan.grasp_pose_6d, T_world_grasp)],
    )
    monkeypatch.setattr(
        planner_module,
        "plan_safe_pick_place_steps",
        lambda **kwargs: fake_plan,
    )
    monkeypatch.setattr(
        planner_module,
        "_load_scene_clearance_obstacles",
        lambda *args: None,
    )

    results = planner_module.plan_pick_place_candidates_from_perception(
        node=object(),
        start_pose_6d=np.zeros(6),
    )

    assert len(results) == 1


def test_planner_fails_closed_when_round_top_scene_clearance_bounds_missing(
    monkeypatch,
):
    _set_round_top_clearance_test_config(monkeypatch)
    fake_plan = SimpleNamespace(
        pre_grasp_pose_6d=np.array([0.0, 0.0, 0.8, 0.0, 0.0, 0.0]),
        grasp_pose_6d=np.array([0.0, 0.0, 0.7, 0.0, 0.0, 0.0]),
        steps=[],
        debug_info={},
    )
    T_world_grasp = np.eye(4)
    T_world_grasp[:3, 3] = fake_plan.grasp_pose_6d[:3]

    monkeypatch.setattr(
        planner_module,
        "estimate_object_world_pose",
        lambda *args: np.eye(4),
    )
    monkeypatch.setattr(
        planner_module,
        "get_selected_object_info",
        lambda *args: "apple",
    )
    monkeypatch.setattr(
        planner_module,
        "select_grasp_pose_candidates_6d",
        lambda *args: [(fake_plan.grasp_pose_6d, T_world_grasp)],
    )
    monkeypatch.setattr(
        planner_module,
        "plan_safe_pick_place_steps",
        lambda **kwargs: fake_plan,
    )
    monkeypatch.setattr(
        planner_module,
        "_load_scene_clearance_obstacles",
        lambda *args: None,
    )

    with pytest.raises(RuntimeError, match="round_top approach clearance requires"):
        planner_module.plan_pick_place_candidates_from_perception(
            node=object(),
            start_pose_6d=np.zeros(6),
        )


def test_planner_rejects_round_top_candidate_blocked_by_scene_object(
    monkeypatch,
):
    _set_round_top_clearance_test_config(monkeypatch)
    fake_plan = SimpleNamespace(
        pre_grasp_pose_6d=np.array([0.0, 0.0, 0.8, 0.0, 0.0, 0.0]),
        grasp_pose_6d=np.array([0.0, 0.0, 0.7, 0.0, 0.0, 0.0]),
        steps=[],
        debug_info={},
    )
    T_world_grasp = np.eye(4)
    T_world_grasp[:3, 3] = fake_plan.grasp_pose_6d[:3]
    obstacle = make_clearance_obstacle(
        "banana",
        center=(0.04, 0.0, 0.75),
        half_extents=(0.02, 0.02, 0.04),
    )

    monkeypatch.setattr(
        planner_module,
        "estimate_object_world_pose",
        lambda *args: np.eye(4),
    )
    monkeypatch.setattr(
        planner_module,
        "get_selected_object_info",
        lambda *args: "apple",
    )
    monkeypatch.setattr(
        planner_module,
        "select_grasp_pose_candidates_6d",
        lambda *args: [(fake_plan.grasp_pose_6d, T_world_grasp)],
    )
    monkeypatch.setattr(
        planner_module,
        "plan_safe_pick_place_steps",
        lambda **kwargs: fake_plan,
    )
    monkeypatch.setattr(
        planner_module,
        "_load_scene_clearance_obstacles",
        lambda *args: [obstacle],
    )

    with pytest.raises(RuntimeError, match="No round_top grasp candidate clears"):
        planner_module.plan_pick_place_candidates_from_perception(
            node=object(),
            start_pose_6d=np.zeros(6),
        )

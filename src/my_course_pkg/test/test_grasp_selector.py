import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from my_course_pkg.grasp import grasp_selector
from my_course_pkg.grasp.config import (
    GRASP_Z_OFFSET,
    SIDE_GRASP_Z_OFFSET,
    TOP_DOWN_GRASP_Z_OFFSET,
    VERTICAL_GRASP_Z_OFFSET,
)


def make_grasp(tool_z_world, translation=(0.0, 0.0, 0.0)):
    tool_z_world = np.asarray(tool_z_world, dtype=float)
    tool_z_world /= np.linalg.norm(tool_z_world)
    ref = np.array([0.0, 0.0, 1.0])
    if abs(float(ref @ tool_z_world)) > 0.9:
        ref = np.array([0.0, 1.0, 0.0])
    tool_x_world = np.cross(ref, tool_z_world)
    tool_x_world /= np.linalg.norm(tool_x_world)
    tool_y_world = np.cross(tool_z_world, tool_x_world)

    transform = np.eye(4)
    transform[:3, :3] = np.column_stack(
        [tool_x_world, tool_y_world, tool_z_world]
    )
    transform[:3, 3] = np.asarray(translation, dtype=float)
    return transform


def make_side_grasp_on_centerline(
    tool_z_world,
    height=0.085,
    geometry_center_xy_in_tcp=(0.0, 0.0),
    selected_object_name="tomato_soup_can",
):
    transform = make_grasp(tool_z_world)
    rotation = transform[:3, :3]
    center_obj = grasp_selector.get_side_grasp_geometry_center(
        selected_object_name,
    )
    center_x, center_y = geometry_center_xy_in_tcp
    center_z = (
        center_obj[2]
        - float(height)
        - rotation[2, 0] * center_x
        - rotation[2, 1] * center_y
    ) / rotation[2, 2]
    center_in_tcp = np.array([center_x, center_y, center_z], dtype=float)
    transform[:3, 3] = center_obj - (rotation @ center_in_tcp)
    return transform


def test_side_centerline_offset_uses_geometry_center_in_tcp():
    grasp = make_side_grasp_on_centerline(
        [np.sqrt(0.5), 0.0, -np.sqrt(0.5)],
        height=0.085,
        geometry_center_xy_in_tcp=(0.012, -0.016),
    )

    assert grasp_selector._object_geometry_center_xy_offset_in_tcp(
        grasp,
        "tomato_soup_can",
    ) == pytest.approx(
        0.02
    )
    assert grasp_selector._object_origin_xy_offset_in_tcp(grasp) > 0.05


def test_can_profile_prefers_centerline_over_slightly_better_height():
    T_world_obj = np.eye(4)
    tool_z = [np.sqrt(0.5), 0.0, -np.sqrt(0.5)]
    off_center_height_match = make_side_grasp_on_centerline(
        tool_z,
        height=0.082,
        geometry_center_xy_in_tcp=(0.08, 0.08),
    )
    centered_slightly_high = make_side_grasp_on_centerline(
        tool_z,
        height=0.09,
        geometry_center_xy_in_tcp=(0.0, 0.0),
    )

    selected = grasp_selector.select_grasp_candidates(
        T_world_obj,
        np.array([off_center_height_match, centered_slightly_high]),
        "tomato_soup_can",
        max_side_candidates=1,
    )

    assert len(selected) == 1
    np.testing.assert_allclose(selected[0], centered_slightly_high)


def test_can_profile_rejects_side_grasps_off_centerline():
    T_world_obj = np.eye(4)
    off_center_grasp = make_side_grasp_on_centerline(
        [np.sqrt(0.5), 0.0, -np.sqrt(0.5)],
        height=0.082,
        geometry_center_xy_in_tcp=(0.08, 0.08),
    )

    with pytest.raises(RuntimeError, match="centerline"):
        grasp_selector.select_grasp_candidates(
            T_world_obj,
            np.array([off_center_grasp]),
            "tomato_soup_can",
            max_side_candidates=1,
        )


def test_default_grasp_root_points_to_workspace_grasps(tmp_path, monkeypatch):
    selected_object = tmp_path / "selected_object.json"
    selected_object.write_text(
        json.dumps({"selected_object_name": "lemon"}),
        encoding="utf-8",
    )

    expected_grasp_dir = "/home/ws/grasps/014_lemon"
    checked_dirs = []

    def fake_isdir(path):
        checked_dirs.append(path)
        return path == expected_grasp_dir

    monkeypatch.setattr(grasp_selector.os.path, "isdir", fake_isdir)

    assert (
        grasp_selector.get_grasp_dir_from_selected_object(selected_object)
        == expected_grasp_dir
    )
    assert checked_dirs == [expected_grasp_dir]


def test_box_profile_prefers_vertical_grasp_over_side_grasp():
    T_world_obj = np.eye(4)
    side_grasp = make_grasp([1.0, 0.0, 0.0])
    vertical_grasp = make_grasp([0.0, 0.0, -1.0])

    selected = grasp_selector.select_preferred_grasp(
        T_world_obj,
        np.array([side_grasp, vertical_grasp]),
        "pudding_box",
    )

    np.testing.assert_allclose(selected, vertical_grasp)


def test_can_profile_prefers_side_grasp_over_vertical_grasp():
    T_world_obj = np.eye(4)
    vertical_grasp = make_grasp([0.0, 0.0, -1.0])
    side_grasp = make_side_grasp_on_centerline(
        [np.sqrt(0.5), 0.0, -np.sqrt(0.5)],
        height=0.085,
    )

    selected = grasp_selector.select_preferred_grasp(
        T_world_obj,
        np.array([vertical_grasp, side_grasp]),
        "tomato_soup_can",
    )

    np.testing.assert_allclose(selected, side_grasp)


def test_can_profile_does_not_fall_back_to_a_top_down_grasp():
    T_world_obj = np.eye(4)
    vertical_grasp = make_grasp([0.0, 0.0, -1.0])

    with pytest.raises(RuntimeError, match="No downward-facing side-grasp"):
        grasp_selector.select_grasp_candidates(
            T_world_obj,
            np.array([vertical_grasp]),
            "tomato_soup_can",
            max_side_candidates=1,
        )


def test_profiles_use_isolated_grasp_offsets():
    assert (
        grasp_selector.get_grasp_z_offset("tomato_soup_can")
        == SIDE_GRASP_Z_OFFSET
    )
    assert (
        grasp_selector.get_grasp_z_offset("foam_brick")
        == VERTICAL_GRASP_Z_OFFSET
    )
    assert (
        grasp_selector.get_grasp_z_offset("hammer")
        == TOP_DOWN_GRASP_Z_OFFSET
    )
    assert grasp_selector.get_grasp_z_offset("banana") == GRASP_Z_OFFSET
    assert SIDE_GRASP_Z_OFFSET == 0.0
    assert VERTICAL_GRASP_Z_OFFSET == 0.0
    assert TOP_DOWN_GRASP_Z_OFFSET == 0.0


@pytest.mark.parametrize("object_name", ["pear", "PEAR", " pear "])
def test_pear_uses_its_own_final_depth_offset(object_name):
    assert grasp_selector.PEAR_GRASP_Z_OFFSET == pytest.approx(0.005)
    assert grasp_selector.get_grasp_z_offset(object_name) == pytest.approx(
        0.005
    )
    assert (
        grasp_selector.get_grasp_z_offset(object_name) - GRASP_Z_OFFSET
        == pytest.approx(0.025)
    )


@pytest.mark.parametrize(
    "object_name",
    sorted(
        name
        for name in grasp_selector.GRASP_PROFILE_BY_OBJECT
        if name != "pear"
    ),
)
def test_non_pear_objects_keep_their_existing_profile_offsets(object_name):
    expected_by_profile = {
        "side": SIDE_GRASP_Z_OFFSET,
        "vertical": VERTICAL_GRASP_Z_OFFSET,
        "top_down": TOP_DOWN_GRASP_Z_OFFSET,
        "round_top": GRASP_Z_OFFSET,
        "centered": GRASP_Z_OFFSET,
    }
    profile = grasp_selector.get_grasp_profile(object_name)

    assert grasp_selector.get_grasp_z_offset(object_name) == pytest.approx(
        expected_by_profile[profile]
    )


def prepare_fixed_grasp_candidate(monkeypatch, object_name):
    candidate = np.eye(4)
    candidate[:3, 3] = [0.20, -0.10, 0.40]
    monkeypatch.setattr(
        grasp_selector,
        "get_selected_object_info",
        lambda _path: object_name,
    )
    monkeypatch.setattr(
        grasp_selector,
        "get_grasp_dir_for_object_name",
        lambda _name: "/unused",
    )
    monkeypatch.setattr(
        grasp_selector,
        "load_grasps_for_object",
        lambda _path: np.array([candidate]),
    )
    monkeypatch.setattr(
        grasp_selector,
        "select_grasp_candidates",
        lambda *_args, **_kwargs: [candidate.copy()],
    )

    prepared = grasp_selector.select_grasp_pose_candidates_6d(
        np.eye(4),
        "unused-selected-object.json",
    )
    return candidate, prepared[0]


def test_prepared_pear_pose_only_changes_world_z_by_dedicated_offset(
    monkeypatch,
):
    candidate, (pose_6d, T_world_grasp) = prepare_fixed_grasp_candidate(
        monkeypatch,
        "pear",
    )

    np.testing.assert_allclose(pose_6d[:2], candidate[:2, 3])
    assert pose_6d[2] == pytest.approx(
        candidate[2, 3] + grasp_selector.PEAR_GRASP_Z_OFFSET
    )
    assert pose_6d[2] - (candidate[2, 3] + GRASP_Z_OFFSET) == pytest.approx(
        0.025
    )
    np.testing.assert_allclose(T_world_grasp[:3, :3], candidate[:3, :3])
    np.testing.assert_allclose(T_world_grasp[:2, 3], candidate[:2, 3])
    assert T_world_grasp[2, 3] == pytest.approx(pose_6d[2])


def test_prepared_apple_pose_keeps_shared_round_top_offset(monkeypatch):
    candidate, (pose_6d, T_world_grasp) = prepare_fixed_grasp_candidate(
        monkeypatch,
        "apple",
    )

    assert pose_6d[2] == pytest.approx(candidate[2, 3] + GRASP_Z_OFFSET)
    np.testing.assert_allclose(T_world_grasp[:3, 3], pose_6d[:3])


def test_hammer_prefers_balanced_candidate_over_more_vertical_handle_end(
    monkeypatch,
    capsys,
):
    monkeypatch.setattr(
        grasp_selector,
        "HAMMER_GRASP_BALANCE_POINT",
        np.zeros(3),
    )
    monkeypatch.setattr(
        grasp_selector,
        "HAMMER_GRASP_BALANCE_WEIGHT",
        250.0,
    )
    monkeypatch.setattr(
        grasp_selector,
        "HAMMER_MAX_TOP_DOWN_ANGLE_DEG",
        10.0,
    )
    handle_end = make_grasp(
        [0.0, 0.0, -1.0],
        translation=(0.10, 0.0, 0.0),
    )
    angle_rad = np.deg2rad(3.0)
    balanced = make_grasp(
        [np.sin(angle_rad), 0.0, -np.cos(angle_rad)],
        translation=(0.001, 0.0, 0.0),
    )

    selected = grasp_selector.select_grasp_candidates(
        np.eye(4),
        np.array([handle_end, balanced]),
        "hammer",
    )

    assert len(selected) == 1
    np.testing.assert_allclose(selected[0], balanced)
    output = capsys.readouterr().out
    assert "Hammer balanced top-down selection" in output
    assert "raw=2" in output
    assert "angle_filtered=2" in output
    assert "selected_balance_distance_m=0.0010" in output


def test_hammer_rejects_near_balance_candidate_above_angle_limit(
    monkeypatch,
):
    monkeypatch.setattr(
        grasp_selector,
        "HAMMER_GRASP_BALANCE_POINT",
        np.zeros(3),
    )
    monkeypatch.setattr(
        grasp_selector,
        "HAMMER_GRASP_BALANCE_WEIGHT",
        250.0,
    )
    monkeypatch.setattr(
        grasp_selector,
        "HAMMER_MAX_TOP_DOWN_ANGLE_DEG",
        10.0,
    )
    steep_angle_rad = np.deg2rad(11.0)
    too_steep = make_grasp(
        [np.sin(steep_angle_rad), 0.0, -np.cos(steep_angle_rad)],
        translation=(0.0, 0.0, 0.0),
    )
    safe_angle_rad = np.deg2rad(2.0)
    safe = make_grasp(
        [np.sin(safe_angle_rad), 0.0, -np.cos(safe_angle_rad)],
        translation=(0.05, 0.0, 0.0),
    )

    selected = grasp_selector.select_grasp_candidates(
        np.eye(4),
        np.array([too_steep, safe]),
        "hammer",
    )

    np.testing.assert_allclose(selected[0], safe)


def test_hammer_fails_closed_without_candidate_inside_angle_limit(
    monkeypatch,
):
    monkeypatch.setattr(
        grasp_selector,
        "HAMMER_MAX_TOP_DOWN_ANGLE_DEG",
        10.0,
    )
    angle_rad = np.deg2rad(11.0)
    too_steep = make_grasp(
        [np.sin(angle_rad), 0.0, -np.cos(angle_rad)],
    )

    with pytest.raises(RuntimeError, match="No hammer top-down candidates"):
        grasp_selector.select_grasp_candidates(
            np.eye(4),
            np.array([too_steep]),
            "hammer",
        )


def test_can_profile_returns_one_candidate_for_each_side_direction():
    T_world_obj = np.eye(4)
    downward_side = 1.0 / np.sqrt(2.0)
    side_grasps = np.array(
        [
            make_side_grasp_on_centerline(
                [downward_side, 0.0, -downward_side],
            ),
            make_side_grasp_on_centerline(
                [-downward_side, 0.0, -downward_side],
            ),
            make_side_grasp_on_centerline(
                [0.0, downward_side, -downward_side],
            ),
            make_side_grasp_on_centerline(
                [0.0, -downward_side, -downward_side],
            ),
        ]
    )

    selected = grasp_selector.select_grasp_candidates(
        T_world_obj,
        side_grasps,
        "tomato_soup_can",
        max_side_candidates=4,
    )

    assert len(selected) == 4
    selected_directions = {
        tuple(np.round(grasp[:3, 2], decimals=6)) for grasp in selected
    }
    assert selected_directions == {
        (0.707107, 0.0, -0.707107),
        (-0.707107, 0.0, -0.707107),
        (0.0, 0.707107, -0.707107),
        (0.0, -0.707107, -0.707107),
    }


def test_can_profile_expands_one_cylindrical_side_grasp_around_object():
    T_world_obj = np.eye(4)
    downward_side = 1.0 / np.sqrt(2.0)
    one_side_grasp = make_side_grasp_on_centerline(
        [downward_side, 0.0, -downward_side],
        height=0.085,
    )

    selected = grasp_selector.select_grasp_candidates(
        T_world_obj,
        np.array([one_side_grasp]),
        "tomato_soup_can",
        max_side_candidates=4,
    )

    assert len(selected) == 4
    center_xy = grasp_selector.get_side_grasp_geometry_center(
        "tomato_soup_can",
    )[:2]
    selected_xy_relative_to_center = {
        tuple(np.round(grasp[:2, 3] - center_xy, decimals=6))
        for grasp in selected
    }
    radius = round(float(np.linalg.norm(one_side_grasp[:2, 3] - center_xy)), 6)
    assert selected_xy_relative_to_center == {
        (-radius, 0.0),
        (0.0, -radius),
        (radius, 0.0),
        (0.0, radius),
    }
    assert all(
        grasp_selector._object_geometry_center_xy_offset_in_tcp(
            grasp,
            "tomato_soup_can",
        )
        == pytest.approx(0.0, abs=1e-8)
        for grasp in selected
    )


def test_can_profile_prefers_tilted_side_grasp_over_level_side_grasp():
    T_world_obj = np.eye(4)
    shallow_side_grasp = make_side_grasp_on_centerline(
        [np.sqrt(0.75), 0.0, -0.5],
        height=0.085,
    )
    tilted_side_grasp = make_side_grasp_on_centerline(
        [np.sqrt(0.5), 0.0, -np.sqrt(0.5)],
        height=0.085,
    )

    selected = grasp_selector.select_grasp_candidates(
        T_world_obj,
        np.array([shallow_side_grasp, tilted_side_grasp]),
        "tomato_soup_can",
        max_side_candidates=1,
    )

    assert len(selected) == 1
    np.testing.assert_allclose(selected[0], tilted_side_grasp)


def test_can_profile_rejects_upward_tilt_that_puts_the_mount_below_tcp():
    T_world_obj = np.eye(4)
    upward_tilt = make_side_grasp_on_centerline(
        [np.sqrt(0.5), 0.0, np.sqrt(0.5)],
        height=0.085,
    )
    downward_tilt = make_side_grasp_on_centerline(
        [np.sqrt(0.5), 0.0, -np.sqrt(0.5)],
        height=0.085,
    )

    selected = grasp_selector.select_grasp_candidates(
        T_world_obj,
        np.array([upward_tilt, downward_tilt]),
        "tomato_soup_can",
        max_side_candidates=1,
    )

    assert len(selected) == 1
    np.testing.assert_allclose(selected[0], downward_tilt)


def test_can_profile_prefers_target_height_tilted_side_grasp():
    T_world_obj = np.eye(4)
    target_height_side_grasp = make_side_grasp_on_centerline(
        [np.sqrt(0.5), 0.0, -np.sqrt(0.5)],
        height=0.085,
    )
    lower_in_range_side_grasp = make_side_grasp_on_centerline(
        [np.sqrt(0.5), 0.0, -np.sqrt(0.5)],
        height=0.065,
    )

    selected = grasp_selector.select_grasp_candidates(
        T_world_obj,
        np.array([lower_in_range_side_grasp, target_height_side_grasp]),
        "tomato_soup_can",
        max_side_candidates=1,
    )

    assert len(selected) == 1
    np.testing.assert_allclose(selected[0], target_height_side_grasp)


def test_can_profile_keeps_high_oblique_grasp_and_rejects_out_of_range_heights():
    T_world_obj = np.eye(4)
    target_height_side_grasp = make_side_grasp_on_centerline(
        [np.sqrt(0.5), 0.0, -np.sqrt(0.5)],
        height=0.085,
    )
    low_side_grasp = make_side_grasp_on_centerline(
        [-np.sqrt(0.5), 0.0, -np.sqrt(0.5)],
        height=0.02,
    )
    high_side_grasp = make_side_grasp_on_centerline(
        [np.sqrt(0.5), 0.0, -np.sqrt(0.5)],
        height=0.12,
    )

    selected = grasp_selector.select_grasp_candidates(
        T_world_obj,
        np.array([low_side_grasp, target_height_side_grasp, high_side_grasp]),
        "tomato_soup_can",
        max_side_candidates=16,
    )

    assert len(selected) == 4
    assert all(grasp[2, 3] == pytest.approx(0.085) for grasp in selected)
    assert any(np.allclose(grasp, target_height_side_grasp) for grasp in selected)


def test_can_profile_default_candidate_limit_keeps_sixteen_high_side_grasps():
    T_world_obj = np.eye(4)
    downward_side = 1.0 / np.sqrt(2.0)
    high_oblique_grasps = np.array(
        [
            make_side_grasp_on_centerline(
                [downward_side, 0.0, -downward_side],
                height=0.085,
                geometry_center_xy_in_tcp=(0.001 * index, 0.0),
            )
            for index in range(4)
        ]
    )

    selected = grasp_selector.select_grasp_candidates(
        T_world_obj,
        high_oblique_grasps,
        "tomato_soup_can",
    )

    assert len(selected) == 16
    assert all(grasp[2, 3] == pytest.approx(0.085) for grasp in selected)


def test_center_profile_prefers_middle_candidate_when_angles_match():
    T_world_obj = np.eye(4)
    end_grasp = make_grasp([0.0, 0.0, -1.0], translation=[0.0, 0.12, 0.0])
    middle_grasp = make_grasp([0.0, 0.0, -1.0], translation=[0.0, 0.01, 0.0])

    selected = grasp_selector.select_preferred_grasp(
        T_world_obj,
        np.array([end_grasp, middle_grasp]),
        "banana",
    )

    np.testing.assert_allclose(selected, middle_grasp)


def make_round_top_grasp(name="apple", angle_tool_z=(0.0, 0.0, -1.0), xy=(0.0, 0.0), normalized_height=0.08):
    geometry = grasp_selector.OBJECT_GEOMETRY_BY_NAME[name]
    center = np.asarray(geometry["center"], dtype=float)
    size = np.asarray(geometry["bbox_size"], dtype=float)
    grasp = make_grasp(angle_tool_z)
    grasp[:3, 3] = center + np.array([xy[0], xy[1], normalized_height * size[2]])
    return grasp


def set_horizontal_closing_axis(grasp, closing_xy):
    closing_xy = np.asarray(closing_xy, dtype=float)
    closing_xy /= np.linalg.norm(closing_xy)
    tool_x = np.array([closing_xy[0], closing_xy[1], 0.0])
    tool_z = np.array([0.0, 0.0, -1.0])
    tool_y = np.cross(tool_z, tool_x)
    result = grasp.copy()
    result[:3, :3] = np.column_stack([tool_x, tool_y, tool_z])
    return result


def select_legacy_round_top_candidates_for_test(
    T_world_obj,
    grasps,
    object_name,
):
    geometry = grasp_selector.OBJECT_GEOMETRY_BY_NAME[object_name]
    center = np.asarray(geometry["center"], dtype=float)
    size = np.asarray(geometry["bbox_size"], dtype=float)
    rotations = [
        grasp_selector._object_z_rotation_about_point(angle, center)
        for angle in grasp_selector.ROUND_TOP_SYMMETRY_YAW_DEG
    ]
    expanded = []
    seen = set()
    for grasp in grasps:
        for rotation in rotations:
            candidate = rotation @ grasp
            key = tuple(np.round(candidate, 8).ravel())
            if key not in seen:
                seen.add(key)
                expanded.append(candidate)
    oriented = [
        grasp
        for grasp in expanded
        if grasp_selector.tool_z_down_angle_deg(T_world_obj @ grasp)
        <= grasp_selector.ROUND_TOP_MAX_APPROACH_ANGLE_DEG
    ]
    centered = [
        grasp
        for grasp in oriented
        if np.linalg.norm(grasp_selector._object_point_in_tcp(grasp, center)[:2])
        <= grasp_selector.ROUND_TOP_MAX_CENTER_OFFSET_M
    ]
    height_ok = [
        grasp
        for grasp in centered
        if grasp_selector.ROUND_TOP_MIN_NORMALIZED_HEIGHT
        <= (grasp[2, 3] - center[2]) / size[2]
        <= grasp_selector.ROUND_TOP_MAX_NORMALIZED_HEIGHT
    ]
    width = float(min(size[0], size[1]))
    width_ok = (
        height_ok
        if width <= grasp_selector.ROUND_TOP_MAX_GRIPPER_OPENING_M
        else []
    )
    bottom_obj = center.copy()
    bottom_obj[2] -= size[2] / 2.0
    table_z = float((T_world_obj @ np.r_[bottom_obj, 1.0])[2])
    grasp_z_offset = grasp_selector.get_grasp_z_offset(object_name)
    clearance_ok = []
    for grasp in width_ok:
        finger_world = (T_world_obj @ grasp) @ np.array(
            [
                0.0,
                0.0,
                grasp_selector.ROUND_TOP_TCP_TO_LOWEST_FINGER_Z_M,
                1.0,
            ]
        )
        if (
            float(finger_world[2] + grasp_z_offset)
            >= table_z + grasp_selector.ROUND_TOP_TABLE_CLEARANCE_M
        ):
            clearance_ok.append(grasp)
    ranked = sorted(
        clearance_ok,
        key=lambda grasp: grasp_selector.score_grasp(
            T_world_obj,
            grasp,
            object_name,
        ),
    )
    return ranked[:grasp_selector.ROUND_TOP_CANDIDATE_COUNT]


def test_category_mapping_preserves_can_and_adds_round_top():
    assert grasp_selector.get_grasp_category("tomato_soup_can") == "cylindrical_can"
    assert grasp_selector.get_grasp_profile("tomato_soup_can") == "side"
    assert grasp_selector.get_grasp_category("apple") == "round_top"
    assert grasp_selector.get_grasp_profile("apple") == "round_top"


def test_default_scene_fixed_objects_cover_all_grasp_categories():
    config_path = (
        Path(__file__).resolve().parents[2]
        / "ifl_air_mujoco_sim"
        / "env"
        / "config"
        / "base_env.yaml"
    )
    config = yaml.safe_load(config_path.read_text())
    fixed_names = config["fixed_object_names"]

    assert fixed_names == [
        "tomato_soup_can",
        "banana",
        "apple",
        "foam_brick",
        "hammer",
    ]
    assert {
        grasp_selector.GRASP_CATEGORY_BY_OBJECT[name]
        for name in fixed_names
    } == {
        "cylindrical_can",
        "banana",
        "round_top",
        "box",
        "tool_top",
    }


def test_unknown_object_fails_closed():
    with pytest.raises(RuntimeError, match="No grasp category"):
        grasp_selector.get_grasp_profile("mystery_object")


def test_round_top_rejects_side_candidate_and_returns_multiple_ranked_candidates():
    good = make_round_top_grasp(normalized_height=0.08)
    second = make_round_top_grasp(normalized_height=0.12)
    side = make_round_top_grasp(angle_tool_z=(1.0, 0.0, 0.0))
    selected = grasp_selector.select_grasp_candidates(
        np.eye(4), np.array([side, second, good]), "apple"
    )
    assert len(selected) >= 2
    np.testing.assert_allclose(selected[0], good)
    assert all(grasp_selector.tool_z_down_angle_deg(g) <= 20.0 for g in selected)


def test_round_top_center_height_and_table_filters_fail_closed():
    off_center = make_round_top_grasp(xy=(0.03, 0.0))
    too_high = make_round_top_grasp(normalized_height=0.5)
    with pytest.raises(RuntimeError, match="round-top candidates"):
        grasp_selector.select_grasp_candidates(
            np.eye(4), np.array([off_center, too_high]), "apple"
        )
    good = make_round_top_grasp()
    with pytest.raises(RuntimeError, match="round-top candidates"):
        grasp_selector.select_grasp_candidates(
            np.eye(4), np.array([good]), "apple", table_z=1.0
        )


def test_round_top_width_filter_rejects_object_larger_than_opening(monkeypatch):
    geometry = dict(grasp_selector.OBJECT_GEOMETRY_BY_NAME["apple"])
    geometry["bbox_size"] = [0.09, 0.09, geometry["bbox_size"][2]]
    monkeypatch.setitem(grasp_selector.OBJECT_GEOMETRY_BY_NAME, "apple", geometry)
    with pytest.raises(RuntimeError, match="round-top candidates"):
        grasp_selector.select_grasp_candidates(
            np.eye(4), np.array([make_round_top_grasp()]), "apple"
        )


def test_round_top_table_clearance_includes_later_world_z_offset(monkeypatch):
    grasp = make_round_top_grasp()
    raw_finger_z = float(
        (grasp @ np.array([0.0, 0.0, grasp_selector.ROUND_TOP_TCP_TO_LOWEST_FINGER_Z_M, 1.0]))[2]
    )
    monkeypatch.setattr(grasp_selector, "GRASP_Z_OFFSET", -0.02, raising=False)
    with pytest.raises(RuntimeError, match="round-top candidates"):
        grasp_selector.select_grasp_candidates(
            np.eye(4), np.array([grasp]), "apple",
            table_z=raw_finger_z - 0.01,
        )


def test_pear_filter_defaults_are_object_namespaced():
    assert grasp_selector.PEAR_MAX_CLOSING_AXIS_ERROR_DEG == pytest.approx(5.0)
    assert grasp_selector.PEAR_MIN_OPENING_MARGIN_M == pytest.approx(0.005)
    assert grasp_selector.PEAR_MAX_FINGER_HEIGHT_DELTA_M == pytest.approx(0.002)
    assert grasp_selector.PEAR_MAX_ORIENTATION_CORRECTION_DEG == pytest.approx(
        20.0
    )
    assert grasp_selector.PEAR_CLOSE_POSITION_TOLERANCE_RAD == pytest.approx(
        0.010
    )


@pytest.mark.parametrize("closing_xy", ([1.0, 0.0], [-1.0, 0.0]))
def test_pear_closing_metrics_treat_short_axis_signs_as_equivalent(closing_xy):
    grasp = set_horizontal_closing_axis(
        make_round_top_grasp("pear"),
        closing_xy,
    )
    size = grasp_selector.OBJECT_GEOMETRY_BY_NAME["pear"]["bbox_size"]

    angle_deg, width_m = grasp_selector._pear_short_axis_candidate_metrics(
        grasp,
        size,
    )

    assert angle_deg == pytest.approx(0.0)
    assert width_m == pytest.approx(0.066546)


def test_pear_closing_metrics_measure_long_axis_and_diagonal_widths():
    size = grasp_selector.OBJECT_GEOMETRY_BY_NAME["pear"]["bbox_size"]
    long_axis = set_horizontal_closing_axis(
        make_round_top_grasp("pear"),
        [0.0, 1.0],
    )
    diagonal = set_horizontal_closing_axis(
        make_round_top_grasp("pear"),
        [1.0, 1.0],
    )

    long_angle, long_width = (
        grasp_selector._pear_short_axis_candidate_metrics(long_axis, size)
    )
    diagonal_angle, diagonal_width = (
        grasp_selector._pear_short_axis_candidate_metrics(diagonal, size)
    )

    assert long_angle == pytest.approx(90.0)
    assert long_width == pytest.approx(0.100455)
    assert diagonal_angle == pytest.approx(45.0)
    assert diagonal_width == pytest.approx(
        (0.066546 + 0.100455) / np.sqrt(2.0)
    )


def test_pear_closing_metrics_fail_closed_for_vertical_closing_axis():
    invalid = make_round_top_grasp("pear")
    invalid[:3, 0] = [0.0, 0.0, 1.0]
    size = grasp_selector.OBJECT_GEOMETRY_BY_NAME["pear"]["bbox_size"]

    with pytest.raises(RuntimeError, match="horizontal closing axis"):
        grasp_selector._pear_short_axis_candidate_metrics(invalid, size)


@pytest.mark.parametrize("closing_sign", [1.0, -1.0])
def test_pear_level_synthesis_preserves_depth_and_centers_both_wrist_signs(
    closing_sign,
):
    center = np.asarray(
        grasp_selector.OBJECT_GEOMETRY_BY_NAME["pear"]["center"],
        dtype=float,
    )
    size = np.asarray(
        grasp_selector.OBJECT_GEOMETRY_BY_NAME["pear"]["bbox_size"],
        dtype=float,
    )
    seed = set_horizontal_closing_axis(
        make_round_top_grasp("pear", xy=(0.006, -0.004)),
        [closing_sign, 0.0],
    )
    tilt_rad = np.deg2rad(15.0)
    tilt_about_tcp_y = np.array(
        [
            [np.cos(tilt_rad), 0.0, np.sin(tilt_rad)],
            [0.0, 1.0, 0.0],
            [-np.sin(tilt_rad), 0.0, np.cos(tilt_rad)],
        ]
    )
    seed[:3, :3] = seed[:3, :3] @ tilt_about_tcp_y
    seed_center_tcp_z = grasp_selector._object_point_in_tcp(seed, center)[2]

    candidate, correction_deg, finger_height_delta_m = (
        grasp_selector._pear_level_centered_candidate(
            np.eye(4),
            seed,
            center,
            size,
        )
    )

    center_tcp = grasp_selector._object_point_in_tcp(candidate, center)
    np.testing.assert_allclose(center_tcp[:2], np.zeros(2), atol=1e-12)
    assert center_tcp[2] == pytest.approx(seed_center_tcp_z)
    np.testing.assert_allclose(candidate[:3, 2], [0.0, 0.0, -1.0], atol=1e-12)
    assert candidate[2, 0] == pytest.approx(0.0, abs=1e-12)
    assert np.sign(candidate[0, 0]) == closing_sign
    assert np.linalg.det(candidate[:3, :3]) == pytest.approx(1.0)
    assert correction_deg == pytest.approx(15.0)
    assert finger_height_delta_m == pytest.approx(0.0)


def test_pear_close_position_uses_candidate_width_and_calibration_tolerance():
    candidate = set_horizontal_closing_axis(
        make_round_top_grasp("pear"),
        [1.0, 0.0],
    )

    metrics = grasp_selector.pear_close_position_metrics(candidate)
    expected = (
        grasp_selector.GRIPPER_CLOSED_POSITION
        * (
            grasp_selector.ROUND_TOP_MAX_GRIPPER_OPENING_M
            - metrics["projected_width_m"]
        )
        / grasp_selector.ROUND_TOP_MAX_GRIPPER_OPENING_M
    )

    assert metrics["alignment_error_deg"] == pytest.approx(0.0)
    assert metrics["projected_width_m"] == pytest.approx(0.066546)
    assert metrics["expected_position_rad"] == pytest.approx(expected)
    assert metrics["minimum_position_rad"] == pytest.approx(
        expected - grasp_selector.PEAR_CLOSE_POSITION_TOLERANCE_RAD
    )


def test_pear_selector_returns_only_short_axis_opening_safe_candidates(capsys):
    long_axis = set_horizontal_closing_axis(
        make_round_top_grasp("pear"),
        [0.0, 1.0],
    )

    selected = grasp_selector.select_grasp_candidates(
        np.eye(4),
        np.array([long_axis]),
        "pear",
    )

    assert len(selected) == 2
    size = grasp_selector.OBJECT_GEOMETRY_BY_NAME["pear"]["bbox_size"]
    for candidate in selected:
        angle_deg, width_m = (
            grasp_selector._pear_short_axis_candidate_metrics(candidate, size)
        )
        assert angle_deg <= grasp_selector.PEAR_MAX_CLOSING_AXIS_ERROR_DEG
        assert (
            grasp_selector.ROUND_TOP_MAX_GRIPPER_OPENING_M - width_m
            >= grasp_selector.PEAR_MIN_OPENING_MARGIN_M
        )
    output = capsys.readouterr().out
    assert (
        "Pear short-axis selection: input=8, direction=2, "
        "seed_width=2, leveled=2"
    ) in output
    assert "alignment_error_deg=0.000" in output
    assert "projected_width_m=0.06655" in output
    assert "opening_margin_m=0.01861" in output
    assert "finger_height_delta_m=0.00000" in output
    assert "center_offset_m=0.00000" in output


def test_pear_selector_fails_closed_when_expansion_cannot_reach_short_axis():
    angle_rad = np.deg2rad(22.5)
    diagonal_only = set_horizontal_closing_axis(
        make_round_top_grasp("pear"),
        [np.cos(angle_rad), np.sin(angle_rad)],
    )

    with pytest.raises(RuntimeError, match="No pear candidates satisfy"):
        grasp_selector.select_grasp_candidates(
            np.eye(4),
            np.array([diagonal_only]),
            "pear",
        )


def test_non_pear_round_top_candidate_order_remains_existing_yaw_order():
    grasp = make_round_top_grasp("apple")
    expected = select_legacy_round_top_candidates_for_test(
        np.eye(4),
        np.array([grasp]),
        "apple",
    )

    selected = grasp_selector.select_grasp_candidates(
        np.eye(4),
        np.array([grasp]),
        "apple",
    )

    assert len(selected) == len(expected)
    for actual, expected_candidate in zip(selected, expected):
        np.testing.assert_allclose(actual, expected_candidate)


@pytest.mark.parametrize(
    "object_name",
    sorted(
        name
        for name, profile in grasp_selector.GRASP_PROFILE_BY_OBJECT.items()
        if profile == "round_top" and name != "pear"
    ),
)
def test_non_pear_round_top_never_invokes_pear_metrics(
    monkeypatch,
    object_name,
):
    def fail_if_called(*_args, **_kwargs):
        raise AssertionError("pear-only metrics called for a non-pear object")

    monkeypatch.setattr(
        grasp_selector,
        "_pear_short_axis_candidate_metrics",
        fail_if_called,
    )
    monkeypatch.setattr(
        grasp_selector,
        "_pear_level_centered_candidate",
        fail_if_called,
    )

    selected = grasp_selector.select_grasp_candidates(
        np.eye(4),
        np.array([make_round_top_grasp(object_name)]),
        object_name,
    )

    assert selected


def test_real_pear_library_synthesizes_only_known_safe_wrist_pair():
    grasp_dir = (
        Path(__file__).resolve().parents[3]
        / "grasps"
        / "016_pear"
    )
    grasps = grasp_selector.load_grasps_for_object(grasp_dir)

    selected = grasp_selector.select_grasp_candidates(
        np.eye(4),
        grasps,
        "pear",
    )

    center = np.asarray(
        grasp_selector.OBJECT_GEOMETRY_BY_NAME["pear"]["center"],
        dtype=float,
    )
    size = np.asarray(
        grasp_selector.OBJECT_GEOMETRY_BY_NAME["pear"]["bbox_size"],
        dtype=float,
    )
    seeds = [
        grasp_selector._object_z_rotation_about_point(yaw, center) @ grasps[5675]
        for yaw in (0, 180)
    ]
    expected = [
        grasp_selector._pear_level_centered_candidate(
            np.eye(4),
            seed,
            center,
            size,
        )[0]
        for seed in seeds
    ]

    assert len(selected) == 2
    assert all(
        any(
            np.allclose(candidate, expected_candidate, atol=1e-10, rtol=0.0)
            for expected_candidate in expected
        )
        for candidate in selected
    )
    np.testing.assert_allclose(selected[0][:3, 0], -selected[1][:3, 0])
    np.testing.assert_allclose(selected[0][:3, 2], selected[1][:3, 2])
    np.testing.assert_allclose(selected[0][:3, 3], selected[1][:3, 3])
    for candidate, seed in zip(selected, seeds):
        angle_deg, width_m = (
            grasp_selector._pear_short_axis_candidate_metrics(candidate, size)
        )
        center_tcp = grasp_selector._object_point_in_tcp(candidate, center)
        seed_center_tcp = grasp_selector._object_point_in_tcp(seed, center)
        assert angle_deg == pytest.approx(0.0, abs=1e-9)
        assert width_m == pytest.approx(0.066546, abs=1e-9)
        np.testing.assert_allclose(center_tcp[:2], np.zeros(2), atol=1e-12)
        assert center_tcp[2] == pytest.approx(seed_center_tcp[2])
        np.testing.assert_allclose(candidate[:3, 2], [0.0, 0.0, -1.0])
        assert (
            grasp_selector.ROUND_TOP_MAX_GRIPPER_OPENING_M
            * abs(candidate[2, 0])
            <= grasp_selector.PEAR_MAX_FINGER_HEIGHT_DELTA_M
        )
        assert (
            grasp_selector.ROUND_TOP_MAX_GRIPPER_OPENING_M - width_m
            >= grasp_selector.PEAR_MIN_OPENING_MARGIN_M
        )

import json
import os
import posixpath

import numpy as np
from sim_pick_place.utils.helpers import _transform_matrix_to_pose6d

from my_course_pkg.grasp.candidate_ranker import (
    load_grasp_candidates,
)
from my_course_pkg.grasp.config import (
    GRASP_CATEGORY_BY_OBJECT,
    GRASP_PROFILE_BY_OBJECT,
    GRASP_ROOT,
    GRASP_Z_OFFSET,
    GRIPPER_CLOSED_POSITION,
    GRIPPER_CLOSE_MIN_POSITION,
    PEAR_CLOSE_POSITION_TOLERANCE_RAD,
    PEAR_GRASP_Z_OFFSET,
    PEAR_MAX_FINGER_HEIGHT_DELTA_M,
    PEAR_MAX_ORIENTATION_CORRECTION_DEG,
    HAMMER_GRASP_BALANCE_POINT,
    HAMMER_GRASP_BALANCE_WEIGHT,
    HAMMER_MAX_TOP_DOWN_ANGLE_DEG,
    SIDE_GRASP_CANDIDATE_COUNT,
    SIDE_GRASP_CENTERLINE_WEIGHT,
    SIDE_GRASP_GEOMETRY_CENTER_BY_OBJECT,
    SIDE_GRASP_HEIGHT_WEIGHT,
    SIDE_GRASP_MAX_CENTERLINE_OFFSET_M,
    SIDE_GRASP_MAX_HEIGHT_M,
    SIDE_GRASP_MAX_SIDE_AXIS_ANGLE_DEG,
    SIDE_GRASP_MAX_TOOL_Z_DOWN_ANGLE_DEG,
    SIDE_GRASP_MIN_TOOL_Z_DOWN_ANGLE_DEG,
    SIDE_GRASP_MIN_HEIGHT_M,
    SIDE_GRASP_PREFERRED_TILT_DEG,
    SIDE_GRASP_STEEP_TILT_PENALTY,
    SIDE_GRASP_SYMMETRY_YAW_DEG,
    SIDE_GRASP_TARGET_HEIGHT_M,
    SIDE_GRASP_Z_OFFSET,
    TOP_DOWN_GRASP_Z_OFFSET,
    VERTICAL_GRASP_Z_OFFSET,
    YCB_GRASP_NAME_MAP,
    OBJECT_GEOMETRY_BY_NAME,
    ROUND_TOP_CANDIDATE_COUNT,
    ROUND_TOP_MAX_APPROACH_ANGLE_DEG,
    ROUND_TOP_MAX_CENTER_OFFSET_M,
    ROUND_TOP_MAX_GRIPPER_OPENING_M,
    ROUND_TOP_MAX_NORMALIZED_HEIGHT,
    ROUND_TOP_MIN_NORMALIZED_HEIGHT,
    ROUND_TOP_SYMMETRY_YAW_DEG,
    ROUND_TOP_TABLE_CLEARANCE_M,
    ROUND_TOP_TARGET_NORMALIZED_HEIGHT,
    ROUND_TOP_TCP_TO_LOWEST_FINGER_Z_M,
    PEAR_MAX_CLOSING_AXIS_ERROR_DEG,
    PEAR_MIN_OPENING_MARGIN_M,
)
from my_course_pkg.grasp.transforms import assert_valid_rotation


WORLD_DOWN = np.array([0.0, 0.0, -1.0])
WORLD_UP = np.array([0.0, 0.0, 1.0])
CENTER_OFFSET_SCORE_WEIGHT = 250.0


def get_selected_object_info(json_path):
    with open(json_path, "r") as f:
        data = json.load(f)
    object_info = data["selected_object_name"].strip().lower().replace(" ", "_")
    print("Selected object name", repr(object_info))
    return object_info


def _join_grasp_path(grasp_root, grasp_folder_name):
    grasp_root = str(grasp_root)
    if grasp_root.startswith("/"):
        return posixpath.join(grasp_root, grasp_folder_name)
    return os.path.join(grasp_root, grasp_folder_name)


def get_grasp_dir_for_object_name(selected_name, grasp_root=None):
    if selected_name not in YCB_GRASP_NAME_MAP:
        raise RuntimeError(
            f"Selected object '{selected_name}' not found in YCB_GRASP_NAME_MAP. "
            f"Available keys: {list(YCB_GRASP_NAME_MAP.keys())}"
        )

    grasp_folder_name = YCB_GRASP_NAME_MAP[selected_name]
    if grasp_root is None:
        grasp_root = GRASP_ROOT
    grasp_dir = _join_grasp_path(grasp_root, grasp_folder_name)

    if not os.path.isdir(grasp_dir):
        raise RuntimeError(
            f"Grasp directory does not exist for selected object '{selected_name}': "
            f"{grasp_dir}"
        )

    return grasp_dir


def get_grasp_dir_from_selected_object(json_path, grasp_root=None):
    selected_name = get_selected_object_info(json_path)
    return get_grasp_dir_for_object_name(selected_name, grasp_root)


def load_grasps_for_object(grasp_dir):
    return load_grasp_candidates(grasp_dir)


def _normalize(vector):
    vector = np.asarray(vector, dtype=float)
    norm = np.linalg.norm(vector)
    if norm == 0.0:
        raise RuntimeError("Cannot normalize a zero-length vector.")
    return vector / norm


def _angle_deg(vector, target):
    vector = _normalize(vector)
    target = _normalize(target)
    cos_angle = float(np.clip(vector @ target, -1.0, 1.0))
    return float(np.degrees(np.arccos(cos_angle)))


def _tool_z_axis_world(transform_matrix):
    return _normalize(transform_matrix[:3, :3] @ np.array([0.0, 0.0, 1.0]))


def tool_z_down_angle_deg(transform_matrix):
    return _angle_deg(_tool_z_axis_world(transform_matrix), WORLD_DOWN)


def _object_side_axes_world(T_world_obj):
    rot = T_world_obj[:3, :3]
    return [
        rot @ np.array([1.0, 0.0, 0.0]),
        rot @ np.array([-1.0, 0.0, 0.0]),
        rot @ np.array([0.0, 1.0, 0.0]),
        rot @ np.array([0.0, -1.0, 0.0]),
    ]


def _min_axis_angle_deg(axis, targets):
    return min(_angle_deg(axis, target) for target in targets)


def _closest_side_axis_index(axis, T_world_obj):
    side_axes = _object_side_axes_world(T_world_obj)
    return int(np.argmin([_angle_deg(axis, target) for target in side_axes]))


def _normalize_object_name(selected_object_name):
    if selected_object_name is None:
        return None
    return selected_object_name.strip().lower().replace(" ", "_")


def get_grasp_profile(selected_object_name):
    selected_object_name = _normalize_object_name(selected_object_name)
    if selected_object_name not in GRASP_PROFILE_BY_OBJECT:
        raise RuntimeError(f"No grasp category is configured for {selected_object_name!r}.")
    return GRASP_PROFILE_BY_OBJECT[selected_object_name]


def get_grasp_category(selected_object_name):
    selected_object_name = _normalize_object_name(selected_object_name)
    if selected_object_name not in GRASP_CATEGORY_BY_OBJECT:
        raise RuntimeError(f"No grasp category is configured for {selected_object_name!r}.")
    return GRASP_CATEGORY_BY_OBJECT[selected_object_name]


def get_grasp_z_offset(selected_object_name):
    selected_object_name = _normalize_object_name(selected_object_name)
    if selected_object_name == "pear":
        return PEAR_GRASP_Z_OFFSET
    profile = get_grasp_profile(selected_object_name)
    if profile == "side":
        return SIDE_GRASP_Z_OFFSET
    if profile == "vertical":
        return VERTICAL_GRASP_Z_OFFSET
    if profile == "top_down":
        return TOP_DOWN_GRASP_Z_OFFSET
    return GRASP_Z_OFFSET


def get_side_grasp_geometry_center(selected_object_name):
    selected_object_name = _normalize_object_name(selected_object_name)
    if selected_object_name is None:
        return np.zeros(3, dtype=float)
    return np.asarray(
        SIDE_GRASP_GEOMETRY_CENTER_BY_OBJECT.get(
            selected_object_name,
            np.zeros(3, dtype=float),
        ),
        dtype=float,
    )


def _object_z_rotation(angle_deg):
    angle_rad = np.deg2rad(float(angle_deg))
    cos_angle = np.cos(angle_rad)
    sin_angle = np.sin(angle_rad)

    transform = np.eye(4)
    transform[:3, :3] = np.array(
        [
            [cos_angle, -sin_angle, 0.0],
            [sin_angle, cos_angle, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    return transform


def _object_translation(offset):
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(offset, dtype=float)
    return transform


def _object_z_rotation_about_point(angle_deg, point):
    point = np.asarray(point, dtype=float)
    return (
        _object_translation(point)
        @ _object_z_rotation(angle_deg)
        @ _object_translation(-point)
    )


def expand_side_symmetric_grasps(grasps, selected_object_name=None):
    if get_grasp_profile(selected_object_name) != "side":
        return list(grasps)

    expanded_grasps = []
    seen = set()
    geometry_center = get_side_grasp_geometry_center(selected_object_name)
    rotations = [
        _object_z_rotation_about_point(angle, geometry_center)
        for angle in SIDE_GRASP_SYMMETRY_YAW_DEG
    ]
    for grasp in grasps:
        for rotation in rotations:
            candidate = rotation @ grasp
            key = tuple(np.round(candidate, decimals=8).ravel())
            if key in seen:
                continue
            seen.add(key)
            expanded_grasps.append(candidate)

    return expanded_grasps


def _object_point_in_tcp(T_obj_grasp, point_obj):
    T_obj_grasp = np.asarray(T_obj_grasp, dtype=float)
    point_obj = np.asarray(point_obj, dtype=float)
    rotation = T_obj_grasp[:3, :3]
    translation = T_obj_grasp[:3, 3]
    return rotation.T @ (point_obj - translation)


def _pear_short_axis_candidate_metrics(T_obj_grasp, bbox_size):
    T_obj_grasp = np.asarray(T_obj_grasp, dtype=float)
    bbox_size = np.asarray(bbox_size, dtype=float)
    closing_xy = T_obj_grasp[:2, 0]
    closing_xy_norm = float(np.linalg.norm(closing_xy))
    if not np.isfinite(closing_xy_norm) or closing_xy_norm <= 1e-9:
        raise RuntimeError(
            "Pear candidate has no valid horizontal closing axis."
        )
    if (
        bbox_size.shape != (3,)
        or not np.isfinite(bbox_size).all()
        or np.any(bbox_size <= 0.0)
    ):
        raise RuntimeError("Pear candidate requires a valid positive bbox size.")

    closing_xy = closing_xy / closing_xy_norm
    short_axis_index = int(np.argmin(bbox_size[:2]))
    short_axis_component = float(abs(closing_xy[short_axis_index]))
    alignment_error_deg = float(
        np.degrees(
            np.arccos(np.clip(short_axis_component, 0.0, 1.0))
        )
    )
    projected_width_m = float(
        np.dot(np.abs(closing_xy), bbox_size[:2])
    )
    return alignment_error_deg, projected_width_m


def _rotation_distance_deg(first_rotation, second_rotation):
    first_rotation = np.asarray(first_rotation, dtype=float)
    second_rotation = np.asarray(second_rotation, dtype=float)
    if (
        first_rotation.shape != (3, 3)
        or second_rotation.shape != (3, 3)
        or not np.isfinite(first_rotation).all()
        or not np.isfinite(second_rotation).all()
    ):
        raise RuntimeError("Pear orientation correction requires finite rotations.")
    relative = first_rotation.T @ second_rotation
    cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def _pear_level_centered_candidate(
    T_world_obj,
    seed_grasp,
    center,
    bbox_size,
):
    T_world_obj = np.asarray(T_world_obj, dtype=float)
    seed_grasp = np.asarray(seed_grasp, dtype=float)
    center = np.asarray(center, dtype=float)
    bbox_size = np.asarray(bbox_size, dtype=float)
    if (
        T_world_obj.shape != (4, 4)
        or seed_grasp.shape != (4, 4)
        or center.shape != (3,)
        or bbox_size.shape != (3,)
        or not np.isfinite(T_world_obj).all()
        or not np.isfinite(seed_grasp).all()
        or not np.isfinite(center).all()
        or not np.isfinite(bbox_size).all()
        or np.any(bbox_size <= 0.0)
    ):
        raise RuntimeError("Pear level synthesis requires finite pose and geometry data.")

    object_rotation = T_world_obj[:3, :3]
    seed_world_rotation = object_rotation @ seed_grasp[:3, :3]
    if (
        abs(float(np.linalg.det(object_rotation)) - 1.0) > 1e-3
        or abs(float(np.linalg.det(seed_world_rotation)) - 1.0) > 1e-3
    ):
        raise RuntimeError("Pear level synthesis requires right-handed rotations.")

    short_axis_index = int(np.argmin(bbox_size[:2]))
    short_axis_object = np.zeros(3, dtype=float)
    short_axis_object[short_axis_index] = 1.0
    seed_sign_value = float(seed_grasp[:3, 0] @ short_axis_object)
    if abs(seed_sign_value) <= 1e-9:
        raise RuntimeError("Pear seed has no usable short-axis closing sign.")

    closing_axis_world = object_rotation @ short_axis_object
    closing_axis_world[2] = 0.0
    horizontal_norm = float(np.linalg.norm(closing_axis_world))
    if not np.isfinite(horizontal_norm) or horizontal_norm <= 1e-9:
        raise RuntimeError("Pear short axis has no valid world-horizontal projection.")
    closing_axis_world /= horizontal_norm
    if seed_sign_value < 0.0:
        closing_axis_world *= -1.0

    tool_z_world = WORLD_DOWN.copy()
    tool_y_world = np.cross(tool_z_world, closing_axis_world)
    tool_y_norm = float(np.linalg.norm(tool_y_world))
    if not np.isfinite(tool_y_norm) or tool_y_norm <= 1e-9:
        raise RuntimeError("Pear level synthesis produced a degenerate TCP-Y axis.")
    tool_y_world /= tool_y_norm
    level_world_rotation = np.column_stack(
        (closing_axis_world, tool_y_world, tool_z_world)
    )
    if abs(float(np.linalg.det(level_world_rotation)) - 1.0) > 1e-6:
        raise RuntimeError("Pear level synthesis produced an invalid rotation.")

    level_object_rotation = object_rotation.T @ level_world_rotation
    seed_center_tcp = _object_point_in_tcp(seed_grasp, center)
    level_center_tcp = np.array([0.0, 0.0, seed_center_tcp[2]], dtype=float)
    candidate = seed_grasp.copy()
    candidate[:3, :3] = level_object_rotation
    candidate[:3, 3] = center - level_object_rotation @ level_center_tcp

    correction_deg = _rotation_distance_deg(
        seed_world_rotation,
        level_world_rotation,
    )
    finger_height_delta_m = float(
        ROUND_TOP_MAX_GRIPPER_OPENING_M * abs(level_world_rotation[2, 0])
    )
    return candidate, correction_deg, finger_height_delta_m


def pear_close_position_metrics(T_obj_grasp, bbox_size=None):
    if bbox_size is None:
        bbox_size = OBJECT_GEOMETRY_BY_NAME["pear"]["bbox_size"]
    alignment_error_deg, projected_width_m = (
        _pear_short_axis_candidate_metrics(T_obj_grasp, bbox_size)
    )
    expected_position_rad = float(
        GRIPPER_CLOSED_POSITION
        * (ROUND_TOP_MAX_GRIPPER_OPENING_M - projected_width_m)
        / ROUND_TOP_MAX_GRIPPER_OPENING_M
    )
    minimum_position_rad = max(
        float(GRIPPER_CLOSE_MIN_POSITION),
        expected_position_rad - PEAR_CLOSE_POSITION_TOLERANCE_RAD,
    )
    return {
        "alignment_error_deg": alignment_error_deg,
        "projected_width_m": projected_width_m,
        "opening_margin_m": (
            ROUND_TOP_MAX_GRIPPER_OPENING_M - projected_width_m
        ),
        "expected_position_rad": expected_position_rad,
        "minimum_position_rad": minimum_position_rad,
    }


def _object_origin_in_tcp(T_obj_grasp):
    return _object_point_in_tcp(T_obj_grasp, np.zeros(3, dtype=float))


def _object_origin_xy_offset_in_tcp(T_obj_grasp):
    return float(np.linalg.norm(_object_origin_in_tcp(T_obj_grasp)[:2]))


def _object_geometry_center_in_tcp(T_obj_grasp, selected_object_name=None):
    return _object_point_in_tcp(
        T_obj_grasp,
        get_side_grasp_geometry_center(selected_object_name),
    )


def _object_geometry_center_xy_offset_in_tcp(
    T_obj_grasp,
    selected_object_name=None,
):
    return float(
        np.linalg.norm(
            _object_geometry_center_in_tcp(
                T_obj_grasp,
                selected_object_name,
            )[:2]
        )
    )


def score_grasp(T_world_obj, T_obj_grasp, selected_object_name=None):
    T_world_grasp = T_world_obj @ T_obj_grasp
    tool_z_world = _tool_z_axis_world(T_world_grasp)
    profile = get_grasp_profile(selected_object_name)

    if profile == "side":
        side_angle = _min_axis_angle_deg(
            tool_z_world,
            _object_side_axes_world(T_world_obj),
        )
        preferred_tilt = SIDE_GRASP_PREFERRED_TILT_DEG
        steep_tilt = max(0.0, side_angle - preferred_tilt)
        grasp_height = float(T_obj_grasp[2, 3])
        height_penalty = (
            abs(grasp_height - SIDE_GRASP_TARGET_HEIGHT_M)
            * SIDE_GRASP_HEIGHT_WEIGHT
        )
        centerline_penalty = (
            _object_geometry_center_xy_offset_in_tcp(
                T_obj_grasp,
                selected_object_name,
            )
            * SIDE_GRASP_CENTERLINE_WEIGHT
        )
        return (
            abs(side_angle - preferred_tilt)
            + steep_tilt * SIDE_GRASP_STEEP_TILT_PENALTY
            + height_penalty
            + centerline_penalty
        )

    if profile == "round_top":
        geometry = OBJECT_GEOMETRY_BY_NAME[selected_object_name]
        center = np.asarray(geometry["center"], dtype=float)
        height = float(geometry["bbox_size"][2])
        normalized_height = (float(T_obj_grasp[2, 3]) - center[2]) / height
        center_offset = float(np.linalg.norm(_object_point_in_tcp(T_obj_grasp, center)[:2]))
        return (tool_z_down_angle_deg(T_world_grasp) + center_offset * 250.0
                + abs(normalized_height - ROUND_TOP_TARGET_NORMALIZED_HEIGHT) * 25.0)

    top_down_score = _angle_deg(tool_z_world, WORLD_DOWN)
    if profile == "top_down" and selected_object_name == "hammer":
        balance_distance = float(
            np.linalg.norm(
                T_obj_grasp[:2, 3]
                - HAMMER_GRASP_BALANCE_POINT[:2]
            )
        )
        return (
            top_down_score
            + balance_distance * HAMMER_GRASP_BALANCE_WEIGHT
        )
    if profile == "centered":
        center_offset = float(np.linalg.norm(T_obj_grasp[:3, 3]))
        return top_down_score + center_offset * CENTER_OFFSET_SCORE_WEIGHT

    return top_down_score


def select_grasp_candidates(
    T_world_obj,
    grasps,
    selected_object_name=None,
    max_side_candidates=None,
    table_z=None,
):
    selected_object_name = _normalize_object_name(selected_object_name)
    profile = get_grasp_profile(selected_object_name)
    raw_grasp_count = len(grasps)
    grasps = expand_side_symmetric_grasps(grasps, selected_object_name)
    ranked_grasps = sorted(
        grasps,
        key=lambda grasp: score_grasp(
            T_world_obj,
            grasp,
            selected_object_name,
        ),
    )
    if not ranked_grasps:
        raise RuntimeError("No grasp poses were loaded.")

    if profile == "round_top":
        geometry = OBJECT_GEOMETRY_BY_NAME[selected_object_name]
        center = np.asarray(geometry["center"], dtype=float)
        size = np.asarray(geometry["bbox_size"], dtype=float)
        rotations = [_object_z_rotation_about_point(a, center) for a in ROUND_TOP_SYMMETRY_YAW_DEG]
        expanded, seen = [], set()
        for grasp in grasps:
            for rotation in rotations:
                candidate = rotation @ grasp
                key = tuple(np.round(candidate, 8).ravel())
                if key not in seen:
                    seen.add(key); expanded.append(candidate)
        oriented = [g for g in expanded if tool_z_down_angle_deg(T_world_obj @ g) <= ROUND_TOP_MAX_APPROACH_ANGLE_DEG]
        centered = [g for g in oriented if np.linalg.norm(_object_point_in_tcp(g, center)[:2]) <= ROUND_TOP_MAX_CENTER_OFFSET_M]
        height_ok = [g for g in centered if ROUND_TOP_MIN_NORMALIZED_HEIGHT <= (g[2, 3]-center[2])/size[2] <= ROUND_TOP_MAX_NORMALIZED_HEIGHT]
        pear_direction_ok = None
        pear_seed_width_ok = None
        pear_final_metrics_by_id = {}
        if selected_object_name == "pear":
            pear_direction_ok = []
            pear_seed_width_ok = []
            for grasp in height_ok:
                alignment_error_deg, projected_width_m = (
                    _pear_short_axis_candidate_metrics(grasp, size)
                )
                if (
                    alignment_error_deg
                    <= PEAR_MAX_CLOSING_AXIS_ERROR_DEG
                ):
                    pear_direction_ok.append(grasp)
                    if (
                        ROUND_TOP_MAX_GRIPPER_OPENING_M
                        - projected_width_m
                        >= PEAR_MIN_OPENING_MARGIN_M
                    ):
                        pear_seed_width_ok.append(grasp)
            if not pear_seed_width_ok:
                raise RuntimeError(
                    "No pear candidates satisfy round-top orientation, "
                    "center, height, short-axis, and opening-margin filters."
                )

            width_ok = []
            for seed in pear_seed_width_ok:
                candidate, correction_deg, finger_height_delta_m = (
                    _pear_level_centered_candidate(
                        T_world_obj,
                        seed,
                        center,
                        size,
                    )
                )
                T_world_candidate = T_world_obj @ candidate
                approach_angle_deg = tool_z_down_angle_deg(T_world_candidate)
                center_offset_m = float(
                    np.linalg.norm(_object_point_in_tcp(candidate, center)[:2])
                )
                normalized_height = float(
                    (candidate[2, 3] - center[2]) / size[2]
                )
                close_metrics = pear_close_position_metrics(candidate, size)
                if not (
                    correction_deg
                    <= PEAR_MAX_ORIENTATION_CORRECTION_DEG
                    and finger_height_delta_m
                    <= PEAR_MAX_FINGER_HEIGHT_DELTA_M
                    and approach_angle_deg
                    <= ROUND_TOP_MAX_APPROACH_ANGLE_DEG
                    and center_offset_m <= ROUND_TOP_MAX_CENTER_OFFSET_M
                    and ROUND_TOP_MIN_NORMALIZED_HEIGHT
                    <= normalized_height
                    <= ROUND_TOP_MAX_NORMALIZED_HEIGHT
                    and close_metrics["alignment_error_deg"]
                    <= PEAR_MAX_CLOSING_AXIS_ERROR_DEG
                    and close_metrics["opening_margin_m"]
                    >= PEAR_MIN_OPENING_MARGIN_M
                ):
                    continue
                width_ok.append(candidate)
                pear_final_metrics_by_id[id(candidate)] = {
                    "correction_deg": correction_deg,
                    "finger_height_delta_m": finger_height_delta_m,
                    "approach_angle_deg": approach_angle_deg,
                    "center_offset_m": center_offset_m,
                    "normalized_height": normalized_height,
                    **close_metrics,
                }
            if not width_ok:
                raise RuntimeError(
                    "No pear candidates satisfy level, center, correction, "
                    "short-axis, and opening-margin filters."
                )
            width = max(
                pear_final_metrics_by_id[id(grasp)]["projected_width_m"]
                for grasp in width_ok
            )
        else:
            width = float(min(size[0], size[1]))
            width_ok = (
                height_ok
                if width <= ROUND_TOP_MAX_GRIPPER_OPENING_M
                else []
            )
        bottom_obj = center.copy(); bottom_obj[2] -= size[2] / 2.0
        inferred_table_z = float((T_world_obj @ np.r_[bottom_obj, 1.0])[2]) if table_z is None else float(table_z)
        clearance_ok = []
        grasp_z_offset = get_grasp_z_offset(selected_object_name)
        for g in width_ok:
            T_world_grasp = T_world_obj @ g
            finger_world = T_world_grasp @ np.array([0.0, 0.0, ROUND_TOP_TCP_TO_LOWEST_FINGER_Z_M, 1.0])
            final_finger_z = float(finger_world[2] + grasp_z_offset)
            if final_finger_z >= inferred_table_z + ROUND_TOP_TABLE_CLEARANCE_M:
                clearance_ok.append(g)
        if not clearance_ok:
            if selected_object_name == "pear":
                raise RuntimeError(
                    "No pear candidates satisfy round-top orientation, "
                    "center, height, short-axis, opening-margin, and "
                    "table-clearance filters."
                )
            raise RuntimeError("No round-top candidates satisfy orientation, center, height, width, and table-clearance filters.")
        ranked = sorted(clearance_ok, key=lambda g: score_grasp(T_world_obj, g, selected_object_name))
        limit = ROUND_TOP_CANDIDATE_COUNT if max_side_candidates is None else max(1, int(max_side_candidates))
        selected = ranked[:limit]
        selected_clearances = [
            float(((T_world_obj @ g) @ np.array([0.0, 0.0, ROUND_TOP_TCP_TO_LOWEST_FINGER_Z_M, 1.0]))[2]
                  + grasp_z_offset - inferred_table_z)
            for g in selected
        ]
        if selected_object_name == "pear":
            selected_metrics = [
                pear_final_metrics_by_id[id(grasp)]
                for grasp in selected
            ]
            selected_metrics_text = "; ".join(
                f"{index}:alignment_error_deg="
                f"{metrics['alignment_error_deg']:.3f},"
                f"projected_width_m={metrics['projected_width_m']:.5f},"
                f"opening_margin_m={metrics['opening_margin_m']:.5f},"
                f"correction_deg={metrics['correction_deg']:.3f},"
                "finger_height_delta_m="
                f"{metrics['finger_height_delta_m']:.5f},"
                f"center_offset_m={metrics['center_offset_m']:.5f},"
                f"expected_close_rad={metrics['expected_position_rad']:.3f},"
                f"minimum_close_rad={metrics['minimum_position_rad']:.3f}"
                for index, metrics in enumerate(
                    selected_metrics,
                    start=1,
                )
            )
            print(
                "Pear short-axis selection: "
                f"input={len(height_ok)}, "
                f"direction={len(pear_direction_ok)}, "
                f"seed_width={len(pear_seed_width_ok)}, "
                f"leveled={len(width_ok)}, "
                f"selected=[{selected_metrics_text}]"
            )
        print(f"Round-top selection: object={selected_object_name}, raw={raw_grasp_count}, symmetry_expanded={len(expanded)}, orientation={len(oriented)}, center={len(centered)}, height={len(height_ok)}, width={len(width_ok)}, table={len(clearance_ok)}, selected={len(selected)}, estimated_width_m={width:.5f}, opening_margin_m={ROUND_TOP_MAX_GRIPPER_OPENING_M-width:.5f}, applied_world_z_offset_m={grasp_z_offset:.5f}, finger_clearance_m=[{min(selected_clearances):.5f},{max(selected_clearances):.5f}]")
        return selected

    if profile == "top_down" and selected_object_name == "hammer":
        top_down_grasps = [
            grasp
            for grasp in ranked_grasps
            if tool_z_down_angle_deg(T_world_obj @ grasp)
            <= HAMMER_MAX_TOP_DOWN_ANGLE_DEG
        ]
        if not top_down_grasps:
            raise RuntimeError(
                "No hammer top-down candidates satisfy the configured "
                "approach-angle limit."
            )
        selected = top_down_grasps[0]
        selected_angle_deg = tool_z_down_angle_deg(T_world_obj @ selected)
        selected_balance_distance_m = float(
            np.linalg.norm(
                selected[:2, 3]
                - HAMMER_GRASP_BALANCE_POINT[:2]
            )
        )
        print(
            "Hammer balanced top-down selection: "
            f"raw={raw_grasp_count}, "
            f"angle_filtered={len(top_down_grasps)}, "
            f"selected_angle_deg={selected_angle_deg:.3f}, "
            "selected_balance_distance_m="
            f"{selected_balance_distance_m:.4f}, "
            "selected_object_translation="
            f"{np.array2string(selected[:3, 3], precision=4)}"
        )
        return [selected]

    if profile != "side":
        return [ranked_grasps[0]]

    side_axes = _object_side_axes_world(T_world_obj)
    side_profile_grasps = [
        grasp
        for grasp in ranked_grasps
        if (
            SIDE_GRASP_MIN_TOOL_Z_DOWN_ANGLE_DEG
            <= tool_z_down_angle_deg(T_world_obj @ grasp)
            <= SIDE_GRASP_MAX_TOOL_Z_DOWN_ANGLE_DEG
            and _min_axis_angle_deg(
                _tool_z_axis_world(T_world_obj @ grasp),
                side_axes,
            )
            <= SIDE_GRASP_MAX_SIDE_AXIS_ANGLE_DEG
        )
    ]
    height_range_grasps = [
        grasp
        for grasp in side_profile_grasps
        if (
            SIDE_GRASP_MIN_HEIGHT_M
            <= float(grasp[2, 3])
            <= SIDE_GRASP_MAX_HEIGHT_M
        )
    ]
    centerline_grasps = [
        grasp
        for grasp in height_range_grasps
        if (
            _object_geometry_center_xy_offset_in_tcp(
                grasp,
                selected_object_name,
            )
            <= SIDE_GRASP_MAX_CENTERLINE_OFFSET_M
        )
    ]
    if height_range_grasps and not centerline_grasps:
        raise RuntimeError(
            "No side-grasp candidates satisfy the TCP "
            "geometry-centerline limit "
            f"{SIDE_GRASP_MAX_CENTERLINE_OFFSET_M:.3f} m for "
            f"{selected_object_name!r}."
        )
    ranked_grasps = centerline_grasps
    print(
        "Side-grasp selection: "
        f"raw={raw_grasp_count}, symmetry_expanded={len(grasps)}, "
        f"side_profile_filter={len(side_profile_grasps)}, "
        f"height_range_filter={len(height_range_grasps)}, "
        f"centerline_filter={len(ranked_grasps)} "
        f"(range=[{SIDE_GRASP_MIN_HEIGHT_M:.3f}, "
        f"{SIDE_GRASP_MAX_HEIGHT_M:.3f}] m, "
        f"max_centerline={SIDE_GRASP_MAX_CENTERLINE_OFFSET_M:.3f} m)"
    )

    candidate_limit = (
        SIDE_GRASP_CANDIDATE_COUNT
        if max_side_candidates is None
        else max(1, int(max_side_candidates))
    )
    selected_by_side = {}
    for grasp in ranked_grasps:
        T_world_grasp = T_world_obj @ grasp
        tool_z_world = _tool_z_axis_world(T_world_grasp)

        side_index = _closest_side_axis_index(tool_z_world, T_world_obj)
        if side_index not in selected_by_side:
            selected_by_side[side_index] = grasp
        if len(selected_by_side) >= candidate_limit:
            break

    if not selected_by_side:
        raise RuntimeError(
            "No downward-facing side-grasp candidates within the object-local "
            "height range "
            "were found for "
            f"{selected_object_name!r}."
        )

    selected_grasps = list(selected_by_side.values())
    for grasp in ranked_grasps:
        if len(selected_grasps) >= candidate_limit:
            break
        if any(np.array_equal(grasp, selected) for selected in selected_grasps):
            continue
        selected_grasps.append(grasp)

    selected_grasps = sorted(
        selected_grasps,
        key=lambda grasp: score_grasp(
            T_world_obj,
            grasp,
            selected_object_name,
        ),
    )
    selected_heights = np.array(
        [float(grasp[2, 3]) for grasp in selected_grasps],
        dtype=float,
    )
    selected_approach_angles = np.array(
        [tool_z_down_angle_deg(T_world_obj @ grasp) for grasp in selected_grasps],
        dtype=float,
    )
    selected_centerline_offsets = np.array(
        [
            _object_geometry_center_xy_offset_in_tcp(
                grasp,
                selected_object_name,
            )
            for grasp in selected_grasps
        ],
        dtype=float,
    )
    print(
        "Side-grasp selection: "
        f"planner_candidates={len(selected_grasps)}, "
        "selected_height_m="
        f"min={selected_heights.min():.4f}, "
        f"max={selected_heights.max():.4f}, "
        f"mean={selected_heights.mean():.4f}; "
        "selected_approach_angle_deg="
        f"min={selected_approach_angles.min():.2f}, "
        f"max={selected_approach_angles.max():.2f}, "
        f"mean={selected_approach_angles.mean():.2f}; "
        "selected_geometry_centerline_offset_m="
        f"min={selected_centerline_offsets.min():.4f}, "
        f"max={selected_centerline_offsets.max():.4f}, "
        f"mean={selected_centerline_offsets.mean():.4f}"
    )
    return selected_grasps


def select_preferred_grasp(T_world_obj, grasps, selected_object_name=None):
    best_grasp = select_grasp_candidates(
        T_world_obj,
        grasps,
        selected_object_name,
        max_side_candidates=1,
    )[0]

    T_world_best = T_world_obj @ best_grasp
    profile = get_grasp_profile(selected_object_name)
    best_score = score_grasp(
        T_world_obj,
        best_grasp,
        selected_object_name,
    )
    print(
        f"Selected grasp profile '{profile}' score: {best_score:.2f}; "
        "tool +Z angle to global -Z: "
        f"{tool_z_down_angle_deg(T_world_best):.2f} deg"
    )
    return best_grasp


def select_grasp_pose_candidates_6d(T_world_obj, selected_object_path):
    selected_object_name = get_selected_object_info(selected_object_path)
    profile = get_grasp_profile(selected_object_name)
    grasp_z_offset = get_grasp_z_offset(selected_object_name)
    grasp_dir = get_grasp_dir_for_object_name(selected_object_name)
    grasps = load_grasps_for_object(grasp_dir)
    T_obj_grasps = select_grasp_candidates(
        T_world_obj,
        grasps,
        selected_object_name,
    )
    candidates = []
    for candidate_index, T_obj_grasp in enumerate(T_obj_grasps, start=1):
        T_world_grasp = T_world_obj @ T_obj_grasp
        grasp_offset = T_world_grasp[:3, 3] - T_world_obj[:3, 3]
        score = score_grasp(
            T_world_obj,
            T_obj_grasp,
            selected_object_name,
        )
        print(
            f"Prepared grasp candidate {candidate_index}/{len(T_obj_grasps)} "
            f"for profile {profile!r}: "
            f"score={score:.2f}, offset="
            f"{np.array2string(grasp_offset, precision=4)}, "
            f"z_offset={grasp_z_offset:.4f} m"
        )
        assert_valid_rotation("T_world_grasp", T_world_grasp)
        grasp_pose_6d = _transform_matrix_to_pose6d(T_world_grasp)
        grasp_pose_6d[2] += grasp_z_offset
        T_world_grasp[:3, 3] = grasp_pose_6d[:3]
        candidates.append((grasp_pose_6d, T_world_grasp))
    if candidates:
        final_world_z = np.array(
            [float(grasp_pose_6d[2]) for grasp_pose_6d, _ in candidates],
            dtype=float,
        )
        print(
            "Selected grasp candidates final world z range: "
            f"min={final_world_z.min():.4f}, "
            f"max={final_world_z.max():.4f}, "
            f"mean={final_world_z.mean():.4f}"
        )
    return candidates


def select_grasp_pose_6d(T_world_obj, selected_object_path):
    return select_grasp_pose_candidates_6d(
        T_world_obj,
        selected_object_path,
    )[0]

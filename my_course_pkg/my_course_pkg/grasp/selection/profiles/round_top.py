"""Round-top and pear-specific grasp selection strategy."""

import numpy as np

from my_course_pkg.grasp.config import (
    GRIPPER_CLOSED_POSITION,
    GRIPPER_CLOSE_MIN_POSITION,
    OBJECT_GEOMETRY_BY_NAME,
    PEAR_CLOSE_POSITION_TOLERANCE_RAD,
    PEAR_MAX_CLOSING_AXIS_ERROR_DEG,
    PEAR_MAX_FINGER_HEIGHT_DELTA_M,
    PEAR_MAX_ORIENTATION_CORRECTION_DEG,
    PEAR_MIN_OPENING_MARGIN_M,
    ROUND_TOP_CANDIDATE_COUNT,
    ROUND_TOP_MAX_APPROACH_ANGLE_DEG,
    ROUND_TOP_MAX_CENTER_OFFSET_M,
    ROUND_TOP_MAX_GRIPPER_OPENING_M,
    ROUND_TOP_MAX_NORMALIZED_HEIGHT,
    ROUND_TOP_SYMMETRY_YAW_DEG,
    ROUND_TOP_TABLE_CLEARANCE_M,
    ROUND_TOP_TCP_TO_LOWEST_FINGER_Z_M,
    get_round_top_min_normalized_height,
)
from my_course_pkg.grasp.selection.catalog import get_grasp_z_offset
from my_course_pkg.grasp.selection.geometry import (
    WORLD_DOWN,
    _object_point_in_tcp,
    _object_z_rotation_about_point,
    tool_z_down_angle_deg,
)
from my_course_pkg.grasp.selection.profiles.scoring import score_grasp


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


def select_round_top_grasp_candidates(
    T_world_obj,
    grasps,
    selected_object_name,
    raw_grasp_count,
    max_side_candidates=None,
    table_z=None,
):
    geometry = OBJECT_GEOMETRY_BY_NAME[selected_object_name]
    center = np.asarray(geometry["center"], dtype=float)
    size = np.asarray(geometry["bbox_size"], dtype=float)
    rotations = [
        _object_z_rotation_about_point(angle, center)
        for angle in ROUND_TOP_SYMMETRY_YAW_DEG
    ]
    expanded, seen = [], set()
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
        if tool_z_down_angle_deg(T_world_obj @ grasp)
        <= ROUND_TOP_MAX_APPROACH_ANGLE_DEG
    ]
    centered = [
        grasp
        for grasp in oriented
        if np.linalg.norm(_object_point_in_tcp(grasp, center)[:2])
        <= ROUND_TOP_MAX_CENTER_OFFSET_M
    ]
    min_normalized_height = get_round_top_min_normalized_height(
        selected_object_name
    )
    normalized_heights = [
        float((grasp[2, 3] - center[2]) / size[2])
        for grasp in centered
    ]
    height_ok = [
        grasp
        for grasp, normalized_height in zip(centered, normalized_heights)
        if min_normalized_height
        <= normalized_height
        <= ROUND_TOP_MAX_NORMALIZED_HEIGHT
    ]
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
            if alignment_error_deg <= PEAR_MAX_CLOSING_AXIS_ERROR_DEG:
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
                correction_deg <= PEAR_MAX_ORIENTATION_CORRECTION_DEG
                and finger_height_delta_m <= PEAR_MAX_FINGER_HEIGHT_DELTA_M
                and approach_angle_deg <= ROUND_TOP_MAX_APPROACH_ANGLE_DEG
                and center_offset_m <= ROUND_TOP_MAX_CENTER_OFFSET_M
                and min_normalized_height
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
    bottom_obj = center.copy()
    bottom_obj[2] -= size[2] / 2.0
    inferred_table_z = (
        float((T_world_obj @ np.r_[bottom_obj, 1.0])[2])
        if table_z is None
        else float(table_z)
    )
    clearance_ok = []
    grasp_z_offset = get_grasp_z_offset(selected_object_name)
    for grasp in width_ok:
        T_world_grasp = T_world_obj @ grasp
        finger_world = T_world_grasp @ np.array(
            [0.0, 0.0, ROUND_TOP_TCP_TO_LOWEST_FINGER_Z_M, 1.0]
        )
        final_finger_z = float(finger_world[2] + grasp_z_offset)
        if final_finger_z >= inferred_table_z + ROUND_TOP_TABLE_CLEARANCE_M:
            clearance_ok.append(grasp)
    if not clearance_ok:
        height_text = (
            "none"
            if not normalized_heights
            else (
                f"[{min(normalized_heights):.3f},"
                f"{max(normalized_heights):.3f}]"
            )
        )
        filter_summary = (
            f"raw={raw_grasp_count}, symmetry_expanded={len(expanded)}, "
            f"orientation={len(oriented)}, center={len(centered)}, "
            f"height={len(height_ok)}, width={len(width_ok)}, "
            f"table={len(clearance_ok)}, "
            f"normalized_height_observed={height_text}, "
            "normalized_height_allowed="
            f"[{min_normalized_height:.3f},"
            f"{ROUND_TOP_MAX_NORMALIZED_HEIGHT:.3f}]"
        )
        if selected_object_name == "pear":
            raise RuntimeError(
                "No pear candidates satisfy round-top orientation, "
                "center, height, short-axis, opening-margin, and "
                f"table-clearance filters ({filter_summary})."
            )
        raise RuntimeError(
            "No round-top candidates satisfy orientation, center, height, "
            f"width, and table-clearance filters ({filter_summary})."
        )
    ranked = sorted(
        clearance_ok,
        key=lambda grasp: score_grasp(
            T_world_obj,
            grasp,
            selected_object_name,
        ),
    )
    limit = (
        ROUND_TOP_CANDIDATE_COUNT
        if max_side_candidates is None
        else max(1, int(max_side_candidates))
    )
    selected = ranked[:limit]
    selected_clearances = [
        float(
            (
                (
                    (T_world_obj @ grasp)
                    @ np.array(
                        [0.0, 0.0, ROUND_TOP_TCP_TO_LOWEST_FINGER_Z_M, 1.0]
                    )
                )[2]
                + grasp_z_offset
                - inferred_table_z
            )
        )
        for grasp in selected
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
    print(
        "Round-top selection: "
        f"object={selected_object_name}, raw={raw_grasp_count}, "
        f"symmetry_expanded={len(expanded)}, orientation={len(oriented)}, "
        f"center={len(centered)}, height={len(height_ok)}, "
        f"width={len(width_ok)}, table={len(clearance_ok)}, "
        "normalized_height_allowed="
        f"[{min_normalized_height:.3f},"
        f"{ROUND_TOP_MAX_NORMALIZED_HEIGHT:.3f}], "
        f"selected={len(selected)}, estimated_width_m={width:.5f}, "
        "opening_margin_m="
        f"{ROUND_TOP_MAX_GRIPPER_OPENING_M - width:.5f}, "
        "applied_world_z_offset_m="
        f"{grasp_z_offset:.5f}, finger_clearance_m="
        f"[{min(selected_clearances):.5f},{max(selected_clearances):.5f}]"
    )
    return selected

"""Shared candidate-scoring rules for each configured grasp profile."""

import numpy as np

from my_course_pkg.grasp.config import (
    HAMMER_GRASP_BALANCE_POINT,
    HAMMER_GRASP_BALANCE_WEIGHT,
    OBJECT_GEOMETRY_BY_NAME,
    ROUND_TOP_TARGET_NORMALIZED_HEIGHT,
    SIDE_GRASP_CENTERLINE_WEIGHT,
    SIDE_GRASP_HEIGHT_WEIGHT,
    SIDE_GRASP_PREFERRED_TILT_DEG,
    SIDE_GRASP_STEEP_TILT_PENALTY,
    SIDE_GRASP_TARGET_HEIGHT_M,
)
from my_course_pkg.grasp.selection.catalog import (
    get_grasp_profile,
    get_side_grasp_geometry_center,
)
from my_course_pkg.grasp.selection.geometry import (
    CENTER_OFFSET_SCORE_WEIGHT,
    WORLD_DOWN,
    _angle_deg,
    _min_axis_angle_deg,
    _object_point_in_tcp,
    _object_side_axes_world,
    _tool_z_axis_world,
    tool_z_down_angle_deg,
)


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
        center_offset = float(
            np.linalg.norm(_object_point_in_tcp(T_obj_grasp, center)[:2])
        )
        return (
            tool_z_down_angle_deg(T_world_grasp)
            + center_offset * 250.0
            + abs(normalized_height - ROUND_TOP_TARGET_NORMALIZED_HEIGHT)
            * 25.0
        )

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

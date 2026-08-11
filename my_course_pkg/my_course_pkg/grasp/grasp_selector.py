import os

import numpy as np
from sim_pick_place.utils.helpers import _transform_matrix_to_pose6d

from my_course_pkg.grasp import config as _config
from my_course_pkg.grasp.candidate_ranker import load_grasp_candidates
from my_course_pkg.grasp.selection.catalog import (
    _join_grasp_path,
    _normalize_object_name,
    get_grasp_category,
    get_grasp_dir_for_object_name,
    get_grasp_dir_from_selected_object,
    get_grasp_profile,
    get_grasp_z_offset,
    get_selected_object_info,
    get_side_grasp_geometry_center,
    load_grasps_for_object,
)
from my_course_pkg.grasp.selection.geometry import (
    CENTER_OFFSET_SCORE_WEIGHT,
    WORLD_DOWN,
    _angle_deg,
    _closest_side_axis_index,
    _min_axis_angle_deg,
    _normalize,
    _object_point_in_tcp,
    _object_side_axes_world,
    _object_translation,
    _object_z_rotation,
    _object_z_rotation_about_point,
    _tool_z_axis_world,
    tool_z_down_angle_deg,
)
from my_course_pkg.grasp.selection.profiles import (
    expand_side_symmetric_grasps,
    score_grasp,
    select_grasp_candidates,
)
from my_course_pkg.grasp.selection.profiles.round_top import (
    _pear_level_centered_candidate,
    _pear_short_axis_candidate_metrics,
    _rotation_distance_deg,
    pear_close_position_metrics,
)
from my_course_pkg.grasp.selection.profiles.scoring import (
    _object_geometry_center_in_tcp,
    _object_geometry_center_xy_offset_in_tcp,
)
from my_course_pkg.grasp.transforms import assert_valid_rotation


__all__ = [
    "CENTER_OFFSET_SCORE_WEIGHT",
    "WORLD_DOWN",
    "_angle_deg",
    "_closest_side_axis_index",
    "_join_grasp_path",
    "_min_axis_angle_deg",
    "_normalize",
    "_normalize_object_name",
    "_object_geometry_center_in_tcp",
    "_object_geometry_center_xy_offset_in_tcp",
    "_object_point_in_tcp",
    "_object_side_axes_world",
    "_object_translation",
    "_object_z_rotation",
    "_object_z_rotation_about_point",
    "_pear_level_centered_candidate",
    "_pear_short_axis_candidate_metrics",
    "_rotation_distance_deg",
    "_tool_z_axis_world",
    "assert_valid_rotation",
    "expand_side_symmetric_grasps",
    "get_grasp_category",
    "get_grasp_dir_for_object_name",
    "get_grasp_dir_from_selected_object",
    "get_grasp_profile",
    "get_grasp_z_offset",
    "get_selected_object_info",
    "get_side_grasp_geometry_center",
    "load_grasp_candidates",
    "load_grasps_for_object",
    "os",
    "pear_close_position_metrics",
    "score_grasp",
    "select_grasp_candidates",
    "select_grasp_pose_6d",
    "select_grasp_pose_candidates_6d",
    "select_preferred_grasp",
    "tool_z_down_angle_deg",
]


def __getattr__(name):
    """Keep historic selector-level configuration imports working."""
    try:
        return getattr(_config, name)
    except AttributeError as error:
        raise AttributeError(
            f"module {__name__!r} has no attribute {name!r}"
        ) from error


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


# Deprecated compatibility wrapper; use ``select_grasp_pose_candidates_6d``
# when callers can consider multiple collision-safe candidates.
def select_grasp_pose_6d(T_world_obj, selected_object_path):
    """Deprecated compatibility wrapper returning the first candidate."""
    return select_grasp_pose_candidates_6d(
        T_world_obj,
        selected_object_path,
    )[0]

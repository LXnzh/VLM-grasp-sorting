"""Selection strategy for side-grasp objects such as cylindrical cans."""

import numpy as np

from my_course_pkg.grasp.config import (
    SIDE_GRASP_CANDIDATE_COUNT,
    SIDE_GRASP_MAX_CENTERLINE_OFFSET_M,
    SIDE_GRASP_MAX_HEIGHT_M,
    SIDE_GRASP_MAX_SIDE_AXIS_ANGLE_DEG,
    SIDE_GRASP_MAX_TOOL_Z_DOWN_ANGLE_DEG,
    SIDE_GRASP_MIN_HEIGHT_M,
    SIDE_GRASP_MIN_TOOL_Z_DOWN_ANGLE_DEG,
    SIDE_GRASP_SYMMETRY_YAW_DEG,
)
from my_course_pkg.grasp.selection.catalog import (
    get_grasp_profile,
    get_side_grasp_geometry_center,
)
from my_course_pkg.grasp.selection.geometry import (
    _closest_side_axis_index,
    _min_axis_angle_deg,
    _object_side_axes_world,
    _object_z_rotation_about_point,
    _tool_z_axis_world,
    tool_z_down_angle_deg,
)
from my_course_pkg.grasp.selection.profiles.scoring import (
    _object_geometry_center_xy_offset_in_tcp,
    score_grasp,
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


def select_side_grasp_candidates(
    T_world_obj,
    ranked_grasps,
    selected_object_name,
    raw_grasp_count,
    expanded_grasp_count,
    max_side_candidates=None,
):
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
        f"raw={raw_grasp_count}, symmetry_expanded={expanded_grasp_count}, "
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

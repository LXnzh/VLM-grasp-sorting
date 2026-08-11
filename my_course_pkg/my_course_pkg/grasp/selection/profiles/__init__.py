"""Dispatch configured objects to their profile-specific selectors."""

from my_course_pkg.grasp.selection.catalog import (
    _normalize_object_name,
    get_grasp_profile,
)
from my_course_pkg.grasp.selection.profiles.round_top import (
    select_round_top_grasp_candidates,
)
from my_course_pkg.grasp.selection.profiles.scoring import score_grasp
from my_course_pkg.grasp.selection.profiles.side import (
    expand_side_symmetric_grasps,
    select_side_grasp_candidates,
)
from my_course_pkg.grasp.selection.profiles.top_down import (
    select_hammer_grasp_candidates,
)


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
        return select_round_top_grasp_candidates(
            T_world_obj,
            grasps,
            selected_object_name,
            raw_grasp_count,
            max_side_candidates=max_side_candidates,
            table_z=table_z,
        )

    if profile == "top_down" and selected_object_name == "hammer":
        return select_hammer_grasp_candidates(
            T_world_obj,
            ranked_grasps,
            raw_grasp_count,
        )

    if profile != "side":
        return [ranked_grasps[0]]

    return select_side_grasp_candidates(
        T_world_obj,
        ranked_grasps,
        selected_object_name,
        raw_grasp_count,
        len(grasps),
        max_side_candidates=max_side_candidates,
    )

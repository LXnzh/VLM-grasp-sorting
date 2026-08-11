"""Top-down strategy for asymmetrical tools such as the hammer."""

import numpy as np

from my_course_pkg.grasp.config import (
    HAMMER_GRASP_BALANCE_POINT,
    HAMMER_MAX_TOP_DOWN_ANGLE_DEG,
)
from my_course_pkg.grasp.selection.geometry import tool_z_down_angle_deg


def select_hammer_grasp_candidates(T_world_obj, ranked_grasps, raw_grasp_count):
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

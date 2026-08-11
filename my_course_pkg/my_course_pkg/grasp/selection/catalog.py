"""Object catalog and grasp-library access helpers."""

import json
import os
import posixpath

import numpy as np

from my_course_pkg.grasp.candidate_ranker import load_grasp_candidates
from my_course_pkg.grasp.config import (
    GRASP_CATEGORY_BY_OBJECT,
    GRASP_PROFILE_BY_OBJECT,
    GRASP_ROOT,
    GRASP_Z_OFFSET,
    PEAR_GRASP_Z_OFFSET,
    SIDE_GRASP_GEOMETRY_CENTER_BY_OBJECT,
    SIDE_GRASP_Z_OFFSET,
    TOP_DOWN_GRASP_Z_OFFSET,
    VERTICAL_GRASP_Z_OFFSET,
    YCB_GRASP_NAME_MAP,
)


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
    """Deprecated compatibility helper retained for external callers."""
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

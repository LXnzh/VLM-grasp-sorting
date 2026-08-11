import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from my_course_pkg.env import env_bool
from my_course_pkg.grasp.config import DROP_POSITION
from my_course_pkg.paths import DEPTH_PATH, RGB_PATH
from my_course_pkg.tasks.sorting.bin_locator import locate_sorting_bins_from_rgbd


@dataclass(frozen=True)
class DropTarget:
    position: np.ndarray
    category: str | None = None
    bin_name: str | None = None
    source: str = "default"


def normalize_category(value):
    return str(value).strip().lower().replace("-", "_").replace(" ", "_")


def is_food_category(value):
    return normalize_category(value) == "food"


def _single_bin_mode_enabled():
    return env_bool("MY_COURSE_SINGLE_BIN_MODE")


def resolve_drop_target(
    selected_object_path,
    node=None,
    camera_frame="camera_orbbec",
    rgb_path=RGB_PATH,
    depth_path=DEPTH_PATH,
    default_position=DROP_POSITION,
    classification_mode=None,
):
    """Choose a VLM category and locate its bin from the saved RGB-D frame.

    ``classification_mode`` is authoritative when explicitly provided. A
    ``None`` value preserves the legacy environment-variable behavior.
    """
    selected_object_path = Path(selected_object_path)
    if selected_object_path.exists():
        with selected_object_path.open("r", encoding="utf-8") as handle:
            selected_payload = json.load(handle)
        raw_category = selected_payload.get("target_category")
        if raw_category is not None:
            category = normalize_category(raw_category)
        else:
            category = None
    else:
        category = None

    enabled = (
        _single_bin_mode_enabled()
        if classification_mode is None
        else bool(classification_mode)
    )
    if not enabled:
        return DropTarget(np.asarray(default_position, dtype=float).copy())
    if category is None:
        raise RuntimeError(
            "Food classification mode is enabled, but selected_object.json "
            "does not contain target_category."
        )

    if not is_food_category(category):
        return DropTarget(
            position=np.asarray(default_position, dtype=float).copy(),
            category=category,
            bin_name="default_non_food",
            source="single_bin_non_food_default",
        )

    if node is None:
        raise RuntimeError(
            "A ROS node with camera TF is required to locate food_bin from RGB-D."
        )

    bins = locate_sorting_bins_from_rgbd(
        node,
        camera_frame,
        rgb_path=rgb_path,
        depth_path=depth_path,
    )

    if "single_bin" not in bins:
        raise RuntimeError("RGB-D detector returned no food_bin in single-bin mode.")
    bin_payload = bins["single_bin"]
    position = np.asarray(bin_payload.get("drop_position"), dtype=float)
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise RuntimeError(
            "RGB-D detector returned an invalid drop position for food_bin: "
            f"{bin_payload.get('drop_position')!r}"
        )
    return DropTarget(
        position=position,
        category=category,
        bin_name=str(bin_payload.get("name", "food_bin")),
        source="single_bin",
    )

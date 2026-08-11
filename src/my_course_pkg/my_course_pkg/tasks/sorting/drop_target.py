"""Resolve immutable world-frame placement targets before PBVS starts."""

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from my_course_pkg.grasp.config import DROP_POSITION

from .locator import locate_sorting_bins_from_rgbd


SORTING_CATEGORIES = frozenset({"food", "non_food"})


@dataclass(frozen=True)
class DropTarget:
    """A validated placement target frozen from the overview observation."""

    position: tuple[float, float, float]
    category: str
    bin_name: str
    observation_stamp: float
    source_frame: str
    rgb_path: str
    depth_path: str
    detection_method: str

    def __post_init__(self):
        position = np.asarray(self.position, dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError(
                "DropTarget.position must contain three finite world values."
            )
        if self.category not in SORTING_CATEGORIES:
            raise ValueError(
                "DropTarget.category must be exactly 'food' or 'non_food'."
            )
        stamp = float(self.observation_stamp)
        if not np.isfinite(stamp) or stamp < 0.0:
            raise ValueError("DropTarget.observation_stamp must be finite.")
        object.__setattr__(self, "position", tuple(float(v) for v in position))
        object.__setattr__(self, "observation_stamp", stamp)

    def as_dict(self):
        """Return JSON-safe provenance for the session and plan diagnostics."""
        return {
            "position_world_m": list(self.position),
            "category": self.category,
            "bin_name": self.bin_name,
            "observation_stamp": self.observation_stamp,
            "source_frame": self.source_frame,
            "rgb_path": self.rgb_path,
            "depth_path": self.depth_path,
            "detection_method": self.detection_method,
        }


def normalize_sorting_category(value):
    """Normalize spelling but reject any category outside the product API."""
    category = str(value).strip().lower().replace("-", "_").replace(" ", "_")
    if category not in SORTING_CATEGORIES:
        raise RuntimeError(
            "VLM classification must be exactly 'food' or 'non_food'; "
            f"got {value!r}."
        )
    return category


def resolve_drop_target(
    category,
    *,
    observation_stamp,
    rgb_path,
    depth_path,
    node=None,
    camera_frame="camera_orbbec",
    default_position=DROP_POSITION,
    diagnostics_dir=None,
):
    """Resolve the placement destination from the pre-PBVS overview frame."""
    category = normalize_sorting_category(category)
    rgb_path = Path(rgb_path)
    depth_path = Path(depth_path)
    if not rgb_path.is_file() or not depth_path.is_file():
        raise RuntimeError(
            "Sorting requires the saved synchronized overview RGB-D pair: "
            f"rgb={rgb_path}, depth={depth_path}."
        )

    if category == "non_food":
        return DropTarget(
            position=tuple(np.asarray(default_position, dtype=float)),
            category=category,
            bin_name="default_non_food",
            observation_stamp=observation_stamp,
            source_frame="world",
            rgb_path=str(rgb_path),
            depth_path=str(depth_path),
            detection_method="configured_non_food_target",
        )

    if node is None:
        raise RuntimeError(
            "A ROS node with camera TF is required to locate food_bin."
        )
    bins = locate_sorting_bins_from_rgbd(
        node,
        camera_frame,
        rgb_path=rgb_path,
        depth_path=depth_path,
        diagnostics_dir=diagnostics_dir,
    )
    bin_payload = bins.get("single_bin")
    if bin_payload is None:
        raise RuntimeError("RGB-D detector returned no food_bin.")
    return DropTarget(
        position=tuple(bin_payload.get("drop_position", ())),
        category=category,
        bin_name=str(bin_payload.get("name", "food_bin")),
        observation_stamp=observation_stamp,
        source_frame=camera_frame,
        rgb_path=str(rgb_path),
        depth_path=str(depth_path),
        detection_method=str(bin_payload.get("detection_method", "unknown")),
    )

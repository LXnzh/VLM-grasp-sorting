"""Mask matching, validation, and ranking for FoundationPose."""

from collections.abc import Callable

import numpy as np


SAM2_CLASS_ALIASES = {
    "tomato soup can": {"tomato soup"},
    "tuna fish can": {"tuna fish"},
}


def canonical_name(name: str) -> str:
    return name.strip().lower().replace("_", " ")


def sam2_class_matches_target(
    class_name: str,
    target: str,
    *,
    aliases: dict[str, set[str]] = SAM2_CLASS_ALIASES,
) -> bool:
    canonical_class = canonical_name(class_name)
    canonical_target = canonical_name(target)
    accepted_names = {canonical_target}
    accepted_names.update(aliases.get(canonical_target, set()))

    class_words = canonical_class.split()
    for accepted_name in accepted_names:
        if canonical_class == accepted_name:
            return True

        target_words = accepted_name.split()
        if len(target_words) > len(class_words):
            continue
        for start in range(len(class_words) - len(target_words) + 1):
            if class_words[start:start + len(target_words)] == target_words:
                return True

    return False


def sam2_detected_class_names(sam2: dict) -> list[str]:
    names = {
        str(annotation.get("class_name", "")).strip()
        for annotation in sam2.get("annotations", [])
        if str(annotation.get("class_name", "")).strip()
    }
    return sorted(names)


def validate_mask_geometry(
    mask: np.ndarray,
    target: str,
    *,
    min_mask_area_px: int,
    mask_border_margin_px: int,
) -> None:
    """Apply the existing minimum-area and complete-object safety checks."""
    area = int(mask.sum())
    if area < min_mask_area_px:
        raise RuntimeError(
            f"SAM2 mask for {target!r} is too small: "
            f"area={area}px, minimum={min_mask_area_px}px."
        )

    ys, xs = np.where(mask)
    if len(xs) == 0:
        raise RuntimeError(f"SAM2 mask for {target!r} is empty.")

    height, width = mask.shape
    margin = max(0, int(mask_border_margin_px))
    touches_border = (
        int(xs.min()) < margin
        or int(ys.min()) < margin
        or int(xs.max()) >= width - margin
        or int(ys.max()) >= height - margin
    )
    if touches_border:
        bbox = (
            int(xs.min()),
            int(ys.min()),
            int(xs.max()),
            int(ys.max()),
        )
        raise RuntimeError(
            f"SAM2 mask for {target!r} is too close to the image border: "
            f"bbox_xyxy={bbox}, image_size=({width}, {height}), "
            f"margin={margin}px. Move the object/camera so the full object "
            "is visible before estimating 6D pose."
        )


def mask_centroid_xy(mask: np.ndarray) -> tuple[int, int] | None:
    ys, xs = np.where(mask)
    if len(xs) == 0:
        return None
    return int(round(float(xs.mean()))), int(round(float(ys.mean())))


def mask_in_pick_exclusion_region(
    mask: np.ndarray,
    exclusion_mask: np.ndarray | None,
    *,
    centroid_for_mask: Callable[[np.ndarray], tuple[int, int] | None] = mask_centroid_xy,
) -> bool:
    if exclusion_mask is None:
        return False
    if mask.shape != exclusion_mask.shape:
        return False
    center = centroid_for_mask(mask)
    if center is None or int(mask.sum()) == 0:
        return False
    x, y = center
    centroid_inside = (
        0 <= y < exclusion_mask.shape[0]
        and 0 <= x < exclusion_mask.shape[1]
        and bool(exclusion_mask[y, x])
    )
    overlap_fraction = float((mask & exclusion_mask).sum()) / float(mask.sum())
    return centroid_inside or overlap_fraction >= 0.25


def annotation_rank_score(annotation: dict) -> float:
    return float(annotation.get("grounding_score", annotation.get("score", 0.0)))


def relative_axis_selected_index(
    usable: list[dict],
    label: str,
    axis: str,
) -> int | None:
    """Return the exact same-class instance selected by relative order."""
    label = str(label or "unknown").strip().lower()
    if label == "unknown":
        return None

    if axis == "horizontal":
        relation_to_rank = {"left": "first", "center": "middle", "right": "last"}
        coordinate_index = 0
    else:
        relation_to_rank = {"top": "first", "middle": "middle", "bottom": "last"}
        coordinate_index = 1

    relation = relation_to_rank.get(label)
    if relation is None:
        return None

    count = len(usable)
    if count == 0:
        return None
    if relation == "middle" and count % 2 == 0:
        raise RuntimeError(
            f"Cannot select a unique middle {axis} instance from "
            f"{count} usable same-class masks."
        )

    ordered_indices = sorted(
        range(count),
        key=lambda index: usable[index]["centroid_xy"][coordinate_index],
    )
    if relation == "first":
        return ordered_indices[0]
    if relation == "last":
        return ordered_indices[-1]
    return ordered_indices[count // 2]


def select_relative_instance(
    usable: list[dict],
    target_region: dict | None,
    *,
    selected_index_for_axis: Callable[[list[dict], str, str], int | None] = (
        relative_axis_selected_index
    ),
) -> list[dict]:
    """Select one instance directly from relative same-class ordering."""
    if not isinstance(target_region, dict):
        return usable

    selected_indices = []
    for axis in ("horizontal", "vertical"):
        selected_index = selected_index_for_axis(
            usable,
            target_region.get(axis),
            axis,
        )
        if selected_index is not None:
            selected_indices.append(selected_index)

    if not selected_indices:
        return usable
    if len(set(selected_indices)) != 1:
        raise RuntimeError(
            "The requested horizontal and vertical relations select "
            "different same-class instances."
        )
    return [usable[selected_indices[0]]]


def target_region_is_user_specified(target_region: dict | None) -> bool:
    if not isinstance(target_region, dict):
        return False
    source = str(target_region.get("source", "")).strip().lower()
    if source != "user_instruction":
        return False
    horizontal = str(target_region.get("horizontal", "unknown")).strip().lower()
    vertical = str(target_region.get("vertical", "unknown")).strip().lower()
    return horizontal != "unknown" or vertical != "unknown"

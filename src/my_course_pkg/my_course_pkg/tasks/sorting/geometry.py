"""Pure RGB-D geometry used to locate the sorting bin.

This module intentionally has no ROS dependencies so the detector can be
tested from recorded images and reused by perception code.
"""

import math
import os

import cv2
import numpy as np

from my_course_pkg.perception.camera import (
    DEFAULT_VERTICAL_FOV_DEG,
    camera_matrix,
)


SINGLE_BIN_MIN_PIXELS = int(
    os.environ.get("MY_COURSE_SINGLE_BIN_MIN_PIXELS", "150")
)
SINGLE_BIN_MIN_HEIGHT_M = float(
    os.environ.get("MY_COURSE_SINGLE_BIN_MIN_HEIGHT_M", "0.02")
)
SINGLE_BIN_RELEASE_CLEARANCE_M = float(
    os.environ.get("MY_COURSE_BIN_RELEASE_CLEARANCE_M", "0.05")
)
SINGLE_BIN_SEARCH_BOUNDS = np.array(
    [
        float(os.environ.get("MY_COURSE_BIN_SEARCH_X_MIN", "-0.80")),
        float(os.environ.get("MY_COURSE_BIN_SEARCH_X_MAX", "-0.40")),
        float(os.environ.get("MY_COURSE_BIN_SEARCH_Y_MIN", "0.28")),
        float(os.environ.get("MY_COURSE_BIN_SEARCH_Y_MAX", "0.58")),
    ],
    dtype=float,
)


def camera_intrinsic(width, height, fovy_deg=51.38):
    """Compatibility wrapper around the shared default camera calibration."""
    if float(fovy_deg) == DEFAULT_VERTICAL_FOV_DEG:
        return camera_matrix(width, height)
    focal = (float(height) / 2.0) / math.tan(math.radians(fovy_deg) / 2.0)
    return np.array(
        [
            [focal, 0.0, float(width) / 2.0],
            [0.0, focal, float(height) / 2.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def _masked_world_points(mask, depth_m, intrinsic, T_world_cv_camera):
    rows, cols = np.where(mask)
    z = depth_m[rows, cols].astype(float)
    valid = np.isfinite(z) & (z > 0.05) & (z < 5.0)
    rows = rows[valid]
    cols = cols[valid]
    z = z[valid]
    if len(z) == 0:
        return np.empty((0, 3), dtype=float)

    fx = float(intrinsic[0, 0])
    fy = float(intrinsic[1, 1])
    cx = float(intrinsic[0, 2])
    cy = float(intrinsic[1, 2])
    camera_points = np.column_stack(
        [
            (cols.astype(float) - cx) * z / fx,
            (rows.astype(float) - cy) * z / fy,
            z,
        ]
    )
    rotation = np.asarray(T_world_cv_camera, dtype=float)[:3, :3]
    translation = np.asarray(T_world_cv_camera, dtype=float)[:3, 3]
    return (rotation @ camera_points.T).T + translation


def _world_coordinate_images(depth_m, intrinsic, T_world_cv_camera):
    """Return world XYZ images for every valid aligned-depth pixel."""
    depth_m = np.asarray(depth_m, dtype=float)
    rows, cols = np.indices(depth_m.shape)
    valid = np.isfinite(depth_m) & (depth_m > 0.05) & (depth_m < 5.0)
    z = depth_m[valid]

    fx = float(intrinsic[0, 0])
    fy = float(intrinsic[1, 1])
    cx = float(intrinsic[0, 2])
    cy = float(intrinsic[1, 2])
    camera_points = np.column_stack(
        [
            (cols[valid].astype(float) - cx) * z / fx,
            (rows[valid].astype(float) - cy) * z / fy,
            z,
        ]
    )
    transform = np.asarray(T_world_cv_camera, dtype=float)
    world_points = camera_points @ transform[:3, :3].T + transform[:3, 3]
    world_images = np.full((*depth_m.shape, 3), np.nan, dtype=float)
    world_images[valid] = world_points
    return world_images


def _world_height_image(depth_m, intrinsic, T_world_cv_camera):
    """Return world Z for every valid depth pixel without using RGB values."""
    return _world_coordinate_images(
        depth_m,
        intrinsic,
        T_world_cv_camera,
    )[:, :, 2]


def _world_xy_roi_mask(
    depth_m,
    intrinsic,
    T_world_cv_camera,
    bounds=SINGLE_BIN_SEARCH_BOUNDS,
):
    """Return pixels inside the qualified food-bin search region."""
    bounds = np.asarray(bounds, dtype=float)
    if (
        bounds.shape != (4,)
        or not np.all(np.isfinite(bounds))
        or bounds[0] >= bounds[1]
        or bounds[2] >= bounds[3]
    ):
        raise ValueError("Food-bin search bounds are invalid.")
    world = _world_coordinate_images(
        depth_m,
        intrinsic,
        T_world_cv_camera,
    )
    return (
        np.isfinite(world[:, :, 0])
        & (world[:, :, 0] >= bounds[0])
        & (world[:, :, 0] <= bounds[1])
        & (world[:, :, 1] >= bounds[2])
        & (world[:, :, 1] <= bounds[3])
    )


def _dominant_support_height(world_height, bin_width_m=0.005):
    """Estimate the tabletop height as the densest horizontal height band."""
    heights = np.asarray(world_height, dtype=float)
    heights = heights[np.isfinite(heights)]
    if len(heights) == 0:
        raise RuntimeError(
            "Could not locate bin: depth image has no valid world points."
        )

    low, high = np.quantile(heights, [0.01, 0.99])
    if high - low < bin_width_m:
        return float(np.median(heights))
    edges = np.arange(low, high + 2.0 * bin_width_m, bin_width_m)
    counts, edges = np.histogram(heights, bins=edges)
    peak = int(np.argmax(counts))
    in_peak = (heights >= edges[peak]) & (heights < edges[peak + 1])
    return float(np.median(heights[in_peak]))


def _geometry_bin_mask(
    depth_m,
    intrinsic,
    T_world_cv_camera,
    min_pixels,
    min_height_m=SINGLE_BIN_MIN_HEIGHT_M,
    allowed_mask=None,
):
    """Find the largest footprint elevated above the dominant table plane."""
    world_height = _world_height_image(depth_m, intrinsic, T_world_cv_camera)
    support_z = _dominant_support_height(world_height)
    elevated = np.isfinite(world_height) & (
        world_height > support_z + min_height_m
    )
    if allowed_mask is not None:
        allowed_mask = np.asarray(allowed_mask, dtype=bool)
        if allowed_mask.shape != elevated.shape:
            raise ValueError("Food-bin ROI/depth shape mismatch.")
        elevated &= allowed_mask

    # Join thin walls split by depth holes and remove isolated depth noise.
    image_scale = max(1, int(round(min(elevated.shape) / 240.0)))
    open_kernel = np.ones((3, 3), dtype=np.uint8)
    close_size = 2 * image_scale + 3
    close_kernel = np.ones((close_size, close_size), dtype=np.uint8)
    cleaned = cv2.morphologyEx(
        elevated.astype(np.uint8), cv2.MORPH_OPEN, open_kernel
    )
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, close_kernel)

    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        cleaned, connectivity=8
    )
    candidates = []
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < int(min_pixels):
            continue
        width = int(stats[label, cv2.CC_STAT_WIDTH])
        height = int(stats[label, cv2.CC_STAT_HEIGHT])
        # Footprint, rather than color or pixel count alone, favors a bin over
        # compact objects that may also be above the table.
        candidates.append((width * height, area, label))

    if not candidates:
        raise RuntimeError(
            "Could not locate single_bin from depth geometry: "
            "no sufficiently large region was at least "
            f"{min_height_m:.3f} m above the support plane."
        )
    _footprint, _area, best_label = max(candidates)
    return labels == best_label, support_z


def _random_color_bin_mask(image_bgr, min_pixels, allowed_mask=None):
    """Find the largest saturated interior region, excluding colored floor."""
    hsv = cv2.cvtColor(np.asarray(image_bgr, dtype=np.uint8), cv2.COLOR_BGR2HSV)
    saturated = (hsv[:, :, 1] >= 90) & (hsv[:, :, 2] >= 70)
    if allowed_mask is not None:
        allowed_mask = np.asarray(allowed_mask, dtype=bool)
        if allowed_mask.shape != saturated.shape:
            raise ValueError("Food-bin ROI/RGB shape mismatch.")
        saturated &= allowed_mask
    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(
        saturated.astype(np.uint8), cv2.MORPH_CLOSE, kernel
    )
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        mask, connectivity=8
    )
    height, width = mask.shape
    candidates = []
    for label in range(1, count):
        x = int(stats[label, cv2.CC_STAT_LEFT])
        y = int(stats[label, cv2.CC_STAT_TOP])
        component_width = int(stats[label, cv2.CC_STAT_WIDTH])
        component_height = int(stats[label, cv2.CC_STAT_HEIGHT])
        area = int(stats[label, cv2.CC_STAT_AREA])
        touches_border = (
            x == 0
            or y == 0
            or x + component_width >= width
            or y + component_height >= height
        )
        if area >= int(min_pixels) and not touches_border:
            candidates.append((area, label))
    if not candidates:
        raise RuntimeError(
            "Could not locate sorting_bin from either depth geometry or "
            "a large saturated region on the table."
        )
    _area, best_label = max(candidates)
    return labels == best_label


def _footprint_centroid_xy(world_points):
    """Get the shape-independent centroid of the XY footprint convex hull."""
    xy = np.asarray(world_points, dtype=float)[:, :2]
    lower = np.quantile(xy, 0.02, axis=0)
    upper = np.quantile(xy, 0.98, axis=0)
    trimmed = xy[np.all((xy >= lower) & (xy <= upper), axis=1)]
    if len(trimmed) < 3:
        return np.median(xy, axis=0)
    hull = cv2.convexHull(trimmed.astype(np.float32)).reshape(-1, 2)
    moments = cv2.moments(hull)
    if abs(moments["m00"]) < 1e-12:
        return np.median(trimmed, axis=0)
    return np.array(
        [moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]],
        dtype=float,
    )


def _detect_bin_from_mask(
    image_bgr,
    depth_m,
    T_world_cv_camera,
    mask,
    bin_name,
    intrinsic=None,
    min_pixels=150,
    release_clearance_m=0.05,
):
    mask_pixels = int(mask.sum())
    if mask_pixels < int(min_pixels):
        raise RuntimeError(
            f"Could not locate {bin_name}: only {mask_pixels} "
            "candidate-mask pixels."
        )
    if intrinsic is None:
        intrinsic = camera_intrinsic(image_bgr.shape[1], image_bgr.shape[0])
    world_points = _masked_world_points(
        mask, np.asarray(depth_m), np.asarray(intrinsic), T_world_cv_camera
    )
    if len(world_points) < int(min_pixels):
        raise RuntimeError(
            f"Could not locate {bin_name} from depth: only "
            f"{len(world_points)} valid points."
        )

    upper = np.quantile(world_points, 0.95, axis=0)
    center_xy = _footprint_centroid_xy(world_points)
    wall_top_z = float(upper[2])
    return {
        "name": bin_name,
        "center_xy": center_xy,
        "wall_top_z": wall_top_z,
        "drop_position": np.array(
            [center_xy[0], center_xy[1], wall_top_z + release_clearance_m],
            dtype=float,
        ),
        "mask_pixels": mask_pixels,
        "depth_points": len(world_points),
    }

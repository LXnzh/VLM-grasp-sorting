"""Diagnostic artifacts for RGB-D sorting-bin selection."""

import json
from pathlib import Path

import cv2
import numpy as np


def _project_world_point(
    point_world,
    intrinsic,
    T_world_cv_camera,
    image_shape,
):
    """Project a world-space release point into the depth image."""
    transform = np.asarray(T_world_cv_camera, dtype=float)
    point_world = np.asarray(point_world, dtype=float)
    point_camera = np.linalg.inv(transform) @ np.append(point_world, 1.0)
    z = float(point_camera[2])
    if not np.isfinite(z) or z <= 0.0:
        return None

    fx = float(intrinsic[0, 0])
    fy = float(intrinsic[1, 1])
    cx = float(intrinsic[0, 2])
    cy = float(intrinsic[1, 2])
    u = int(round(fx * float(point_camera[0]) / z + cx))
    v = int(round(fy * float(point_camera[1]) / z + cy))
    height, width = image_shape[:2]
    if not (0 <= u < width and 0 <= v < height):
        return None
    return {"u": u, "v": v}


def _depth_visualization(depth_m):
    """Create a contrast-normalized, false-color visualization of raw depth."""
    depth_m = np.asarray(depth_m, dtype=float)
    valid = np.isfinite(depth_m) & (depth_m > 0.05) & (depth_m < 5.0)
    normalized = np.zeros(depth_m.shape, dtype=np.uint8)
    if np.any(valid):
        low, high = np.quantile(depth_m[valid], [0.02, 0.98])
        if high <= low:
            high = low + 1e-6
        normalized[valid] = np.clip(
            (depth_m[valid] - low) * 255.0 / (high - low),
            0.0,
            255.0,
        ).astype(np.uint8)
    visualization = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    visualization[~valid] = (0, 0, 0)
    return visualization


def write_sorting_bin_depth_diagnostics(
    depth_m,
    intrinsic,
    T_world_cv_camera,
    bin_mask,
    bin_payload,
    output_dir,
    rgb_path=None,
    depth_path=None,
):
    """Save an annotated depth map and machine-readable drop-point record.

    The marker is the image projection of the final 3-D release point. The
    cyan fill and yellow contour show the depth-derived bin region from which
    its XY center and wall-top height were estimated.
    """
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    depth_m = np.asarray(depth_m, dtype=np.float32)
    bin_mask = np.asarray(bin_mask, dtype=bool)
    if bin_mask.shape != depth_m.shape:
        raise ValueError(
            "Sorting-bin mask/depth shape mismatch: "
            f"{bin_mask.shape} vs {depth_m.shape}."
        )

    selection_pixel = _project_world_point(
        bin_payload["drop_position"],
        intrinsic,
        T_world_cv_camera,
        depth_m.shape,
    )
    depth_bgr = _depth_visualization(depth_m)

    # Make the region that contributed to the choice visible without hiding
    # the underlying depth gradients.
    tint = np.full_like(depth_bgr, (255, 255, 0))  # cyan in BGR
    depth_bgr[bin_mask] = cv2.addWeighted(
        depth_bgr[bin_mask], 0.45, tint[bin_mask], 0.55, 0.0
    )
    contours, _ = cv2.findContours(
        bin_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    cv2.drawContours(depth_bgr, contours, -1, (0, 255, 255), 2)

    if selection_pixel is not None:
        center = (selection_pixel["u"], selection_pixel["v"])
        cv2.drawMarker(
            depth_bgr,
            center,
            (0, 255, 0),
            markerType=cv2.MARKER_CROSS,
            markerSize=24,
            thickness=2,
            line_type=cv2.LINE_AA,
        )
        cv2.circle(depth_bgr, center, 12, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(
            depth_bgr,
            "selected drop point",
            (center[0] + 16, max(24, center[1] - 16)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
    else:
        cv2.putText(
            depth_bgr,
            "selected drop point is outside this camera view",
            (16, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    image_path = output_dir / "depth_drop_selection.png"
    metadata_path = output_dir / "drop_selection.json"
    if not cv2.imwrite(str(image_path), depth_bgr):
        raise OSError(f"Could not write sorting depth diagnostic: {image_path}")

    drop_position = np.asarray(bin_payload["drop_position"], dtype=float)
    metadata = {
        "coordinate_frame": "world",
        "visualization": {
            "image": str(image_path),
            "bin_region": "cyan fill with yellow contour",
            "selected_drop_point": "green cross with white ring",
        },
        "input": {
            "rgb_path": None if rgb_path is None else str(rgb_path),
            "depth_path": None if depth_path is None else str(depth_path),
        },
        "detection_method": str(bin_payload["detection_method"]),
        "support_height_m": float(bin_payload["support_height"]),
        "wall_top_height_m": float(bin_payload["wall_top_z"]),
        "release_clearance_m": float(
            drop_position[2] - float(bin_payload["wall_top_z"])
        ),
        "drop_position_world_m": drop_position.tolist(),
        "selection_pixel": selection_pixel,
        "mask_pixels": int(bin_payload["mask_pixels"]),
        "valid_depth_points": int(bin_payload["depth_points"]),
    }
    with metadata_path.open("w", encoding="utf-8") as handle:
        json.dump(metadata, handle, indent=2, sort_keys=True)
        handle.write("\n")
    return {
        "depth_visualization": str(image_path),
        "metadata": str(metadata_path),
    }

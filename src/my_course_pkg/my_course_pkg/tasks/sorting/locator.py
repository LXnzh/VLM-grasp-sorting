"""High-level sorting-bin detection and ROS RGB-D loading."""

import numpy as np
import cv2

from my_course_pkg.paths import DEPTH_PATH, RGB_PATH, SORTING_OUTPUT_DIR

from .diagnostics import write_sorting_bin_depth_diagnostics
from .geometry import (
    SINGLE_BIN_MIN_PIXELS,
    SINGLE_BIN_RELEASE_CLEARANCE_M,
    _detect_bin_from_mask,
    _dominant_support_height,
    _geometry_bin_mask,
    _random_color_bin_mask,
    _world_height_image,
    _world_xy_roi_mask,
    camera_intrinsic,
)


def _transform_bin_result(result, T_output_detection):
    """Transform one detected bin from its search frame into the output frame."""
    transform = np.asarray(T_output_detection, dtype=float)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError("Sorting-bin output transform must be a finite 4x4 matrix.")
    output_up = transform[:3, :3] @ np.array([0.0, 0.0, 1.0])
    if not np.allclose(output_up, [0.0, 0.0, 1.0], atol=1e-5):
        raise RuntimeError(
            "Sorting-bin detection and output frames must share the same +Z axis."
        )

    def transform_point(point):
        return (transform @ np.append(np.asarray(point, dtype=float), 1.0))[:3]

    center_xy = np.asarray(result["center_xy"], dtype=float)
    support_point = transform_point([*center_xy, result["support_height"]])
    wall_top_point = transform_point([*center_xy, result["wall_top_z"]])
    transformed = dict(result)
    transformed["center_xy"] = wall_top_point[:2]
    transformed["support_height"] = float(support_point[2])
    transformed["wall_top_z"] = float(wall_top_point[2])
    transformed["drop_position"] = transform_point(result["drop_position"])
    return transformed


def detect_sorting_bins(
    image_bgr,
    depth_m,
    T_detection_cv_camera,
    intrinsic=None,
    min_pixels=150,
    release_clearance_m=SINGLE_BIN_RELEASE_CLEARANCE_M,
    diagnostics_dir=None,
    rgb_path=None,
    depth_path=None,
    T_output_detection=None,
):
    """Estimate the bin in a qualified search frame, optionally transforming it."""
    if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ValueError("Sorting-bin RGB image must have shape HxWx3.")
    if depth_m.shape != image_bgr.shape[:2]:
        raise ValueError(
            "RGB/depth shape mismatch: "
            f"{image_bgr.shape[:2]} vs {depth_m.shape}."
        )
    if intrinsic is None:
        intrinsic = camera_intrinsic(image_bgr.shape[1], image_bgr.shape[0])

    required_pixels = max(min_pixels, SINGLE_BIN_MIN_PIXELS)
    search_mask = _world_xy_roi_mask(
        depth_m,
        intrinsic,
        T_detection_cv_camera,
    )
    if int(search_mask.sum()) < required_pixels:
        raise RuntimeError(
            "The qualified food-bin search region has too few visible "
            f"RGB-D pixels: {int(search_mask.sum())}."
        )
    try:
        world_height = _world_height_image(
            depth_m,
            intrinsic,
            T_detection_cv_camera,
        )
        support_z = _dominant_support_height(world_height)
        mask = _random_color_bin_mask(
            image_bgr,
            required_pixels,
            allowed_mask=search_mask,
        )
        detection_method = "random_color_rgbd"
    except RuntimeError as color_error:
        mask, support_z = _geometry_bin_mask(
            depth_m,
            intrinsic,
            T_detection_cv_camera,
            min_pixels=required_pixels,
            allowed_mask=search_mask,
        )
        detection_method = "depth_geometry_fallback"
        print(f"Random-color bin detection failed: {color_error}")
    result = _detect_bin_from_mask(
        image_bgr,
        depth_m,
        T_detection_cv_camera,
        mask,
        "food_bin",
        intrinsic=intrinsic,
        min_pixels=required_pixels,
        release_clearance_m=release_clearance_m,
    )
    result["support_height"] = support_z
    result["detection_method"] = detection_method
    diagnostic_transform = np.asarray(T_detection_cv_camera, dtype=float)
    if T_output_detection is not None:
        result = _transform_bin_result(result, T_output_detection)
        diagnostic_transform = (
            np.asarray(T_output_detection, dtype=float)
            @ diagnostic_transform
        )
    if diagnostics_dir is not None:
        result["diagnostics"] = write_sorting_bin_depth_diagnostics(
            depth_m,
            intrinsic,
            diagnostic_transform,
            mask,
            result,
            diagnostics_dir,
            rgb_path=rgb_path,
            depth_path=depth_path,
        )
    return {"single_bin": result}


def locate_sorting_bins_from_rgbd(
    node,
    camera_frame,
    rgb_path=RGB_PATH,
    depth_path=DEPTH_PATH,
    diagnostics_dir=SORTING_OUTPUT_DIR,
):
    """Detect in MuJoCo/base coordinates and return world-frame bin poses."""
    from my_course_pkg.grasp.transforms import (
        camera_pose_convention_transform,
        get_transform_checked,
    )

    print(f"Sorting-bin synchronized input: rgb={rgb_path}, depth={depth_path}")
    image_bgr = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
    if image_bgr is None:
        raise RuntimeError(f"Could not read sorting-bin RGB image: {rgb_path}")
    depth_m = np.load(depth_path).astype(np.float32)
    detection_frame = "base_link"
    T_detection_tf_camera = get_transform_checked(
        node,
        camera_frame,
        detection_frame,
    )
    T_detection_cv_camera = (
        T_detection_tf_camera @ camera_pose_convention_transform()
    )
    T_world_detection = get_transform_checked(
        node,
        detection_frame,
        "world",
    )
    bins = detect_sorting_bins(
        image_bgr,
        depth_m,
        T_detection_cv_camera,
        diagnostics_dir=diagnostics_dir,
        rgb_path=rgb_path,
        depth_path=depth_path,
        T_output_detection=T_world_detection,
    )
    for category, payload in bins.items():
        print(
            f"RGB-D sorting bin {category!r}: "
            "drop_position="
            f"{np.array2string(payload['drop_position'], precision=4)}, "
            f"mask_pixels={payload['mask_pixels']}, "
            f"depth_points={payload['depth_points']}"
        )
        diagnostics = payload.get("diagnostics")
        if diagnostics is not None:
            print(
                "Saved depth-based drop-point explanation: "
                f"{diagnostics['depth_visualization']}"
            )
    return bins

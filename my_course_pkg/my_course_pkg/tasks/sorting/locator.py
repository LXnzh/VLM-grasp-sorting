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
    camera_intrinsic,
)


def detect_sorting_bins(
    image_bgr,
    depth_m,
    T_world_cv_camera,
    intrinsic=None,
    min_pixels=150,
    release_clearance_m=SINGLE_BIN_RELEASE_CLEARANCE_M,
    diagnostics_dir=None,
    rgb_path=None,
    depth_path=None,
):
    """Estimate the single bin center and height from aligned RGB-D data."""
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
    try:
        mask, support_z = _geometry_bin_mask(
            depth_m,
            intrinsic,
            T_world_cv_camera,
            min_pixels=required_pixels,
        )
        detection_method = "depth_geometry"
    except RuntimeError as geometry_error:
        world_height = _world_height_image(
            depth_m,
            intrinsic,
            T_world_cv_camera,
        )
        support_z = _dominant_support_height(world_height)
        mask = _random_color_bin_mask(image_bgr, required_pixels)
        detection_method = "random_color_rgbd_fallback"
        print(
            "Depth-only bin detection failed; using random-color RGB-D "
            f"mask: {geometry_error}"
        )
    result = _detect_bin_from_mask(
        image_bgr,
        depth_m,
        T_world_cv_camera,
        mask,
        "food_bin",
        intrinsic=intrinsic,
        min_pixels=required_pixels,
        release_clearance_m=release_clearance_m,
    )
    result["support_height"] = support_z
    result["detection_method"] = detection_method
    if diagnostics_dir is not None:
        result["diagnostics"] = write_sorting_bin_depth_diagnostics(
            depth_m,
            intrinsic,
            T_world_cv_camera,
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
    """Load a frame and transform depth-derived bin poses into world."""
    from my_course_pkg.grasp.transforms import (
        camera_pose_convention_transform,
        get_transform_checked,
    )

    print(f"Sorting-bin synchronized input: rgb={rgb_path}, depth={depth_path}")
    image_bgr = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
    if image_bgr is None:
        raise RuntimeError(f"Could not read sorting-bin RGB image: {rgb_path}")
    depth_m = np.load(depth_path).astype(np.float32)
    T_world_tf_camera = get_transform_checked(node, camera_frame, "world")
    T_world_cv_camera = T_world_tf_camera @ camera_pose_convention_transform()
    bins = detect_sorting_bins(
        image_bgr,
        depth_m,
        T_world_cv_camera,
        diagnostics_dir=diagnostics_dir,
        rgb_path=rgb_path,
        depth_path=depth_path,
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

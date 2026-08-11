from dataclasses import dataclass
import re
import threading

import numpy as np
import tf_transformations

from my_course_pkg.grasp.config import (
    APPLE_SCENE_BOUNDS_CENTER_MAX_ERROR_M,
    CAMERA_FRAME,
    FOUNDATIONPOSE_OBJECT_CAMERA_POSE_JSON,
    GRASP_SCENE_CLEARANCE_TOPIC,
    GRASP_SCENE_DESCRIPTION_TIMEOUT_SEC,
    GRASP_TABLETOP_CANONICAL_TILT_WARN_DEG,
    OBJECT_GEOMETRY_BY_NAME,
    ROUND_TOP_APPROACH_CLEARANCE_MARGIN_M,
    ROUND_TOP_APPROACH_CORRIDOR_RADIUS_M,
    ROUND_TOP_APPROACH_VERTICAL_MARGIN_M,
    ROUND_TOP_CLEARANCE_ENABLED,
    ROUND_TOP_CLEARANCE_REQUIRE_SCENE,
    SELECTED_OBJECT_PATH,
    SIDE_GRASP_APPROACH_CLEARANCE_MARGIN_M,
    SIDE_GRASP_APPROACH_CORRIDOR_RADIUS_M,
    SIDE_GRASP_APPROACH_VERTICAL_MARGIN_M,
    SIDE_GRASP_CLEARANCE_ENABLED,
    SIDE_GRASP_MIN_WORLD_Z_ABOVE_OBJECT_M,
    VERTICAL_MIN_TCP_ABOVE_TARGET_BOTTOM_M,
)
from my_course_pkg.grasp.grasp_selector import (
    get_grasp_profile,
    get_grasp_z_offset,
    get_side_grasp_geometry_center,
    get_selected_object_info,
    pear_close_position_metrics,
    select_grasp_pose_candidates_6d,
)
from my_course_pkg.grasp.trajectory_planner import (
    PickPlacePlan,
    build_drop_pose_6d,
    plan_safe_pick_place_steps,
)
from my_course_pkg.grasp.transforms import (
    assert_valid_rotation,
    canonicalize_tabletop_object_pose,
    estimate_object_world_pose,
    get_transform_checked,
    object_local_z_axis_world,
    object_z_tilt_deg,
    pose_text,
    print_pose_summary,
    should_canonicalize_tabletop_object_pose,
)
from my_course_pkg.tasks.sorting.drop_target import resolve_drop_target


CLEARANCE_NUMERIC_EPSILON_M = 1e-9
_RIGID_TRANSFORM_ATOL = 1e-6
_BOUNDS_Z_ALIGNMENT_ATOL = 1e-6


@dataclass
class PickPlacePlanningResult:
    plan: PickPlacePlan
    T_world_obj: np.ndarray
    T_world_grasp: np.ndarray
    grasp_pose_6d: np.ndarray
    candidate_index: int = 0
    candidate_count: int = 1


@dataclass(frozen=True)
class SceneClearanceObstacle:
    name: str
    center: np.ndarray
    radius_xy: float
    half_height: float
    T_world_bounds: np.ndarray = None
    half_extents: np.ndarray = None


@dataclass(frozen=True)
class ApproachClearanceViolation:
    obstacle: SceneClearanceObstacle
    box_xy_distance_m: float
    required_clearance_m: float
    clearance_m: float
    bounds_local_z_min: float
    bounds_local_z_max: float
    overlap_t_enter: float
    overlap_t_exit: float


@dataclass(frozen=True)
class ApproachClearanceConfig:
    profile: str
    enabled: bool
    require_scene: bool
    corridor_radius_m: float
    clearance_margin_m: float
    vertical_margin_m: float


@dataclass(frozen=True)
class AppleBoundsCanonicalization:
    bounds_center_world: np.ndarray
    foundationpose_geometry_center_world: np.ndarray
    center_error_m: float
    canonical_origin_world: np.ndarray


def _approach_clearance_config(grasp_profile):
    if grasp_profile == "side":
        return ApproachClearanceConfig(
            profile=grasp_profile,
            enabled=SIDE_GRASP_CLEARANCE_ENABLED,
            require_scene=False,
            corridor_radius_m=SIDE_GRASP_APPROACH_CORRIDOR_RADIUS_M,
            clearance_margin_m=SIDE_GRASP_APPROACH_CLEARANCE_MARGIN_M,
            vertical_margin_m=SIDE_GRASP_APPROACH_VERTICAL_MARGIN_M,
        )
    if grasp_profile == "round_top":
        return ApproachClearanceConfig(
            profile=grasp_profile,
            enabled=ROUND_TOP_CLEARANCE_ENABLED,
            require_scene=ROUND_TOP_CLEARANCE_REQUIRE_SCENE,
            corridor_radius_m=ROUND_TOP_APPROACH_CORRIDOR_RADIUS_M,
            clearance_margin_m=ROUND_TOP_APPROACH_CLEARANCE_MARGIN_M,
            vertical_margin_m=ROUND_TOP_APPROACH_VERTICAL_MARGIN_M,
        )
    return None


def _object_name_key(value):
    key = str(value or "").strip().lower().replace("-", "_").replace(" ", "_")
    return re.sub(r"^\d+_", "", key)


def _is_target_marker(marker_name, selected_object_name):
    return _object_name_key(marker_name) == _object_name_key(selected_object_name)


def _marker_obstacle_name(marker):
    text = str(getattr(marker, "text", "") or "").strip()
    if text:
        return text
    ns = str(getattr(marker, "ns", "") or "").strip()
    marker_id = getattr(marker, "id", "unknown")
    if ns:
        return f"{ns}/{marker_id}"
    return f"marker_{marker_id}"


def _marker_frame_id(marker):
    header = getattr(marker, "header", None)
    return str(getattr(header, "frame_id", "") or "world").strip() or "world"


def _validated_rigid_transform(name, value, use_shared_rotation_check=False):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4):
        raise RuntimeError(f"{name} must be a 4x4 transform.")
    if not np.all(np.isfinite(matrix)):
        raise RuntimeError(f"{name} contains NaN or Inf.")
    if not np.allclose(
        matrix[3, :],
        [0.0, 0.0, 0.0, 1.0],
        atol=CLEARANCE_NUMERIC_EPSILON_M,
        rtol=0.0,
    ):
        raise RuntimeError(f"{name} has an invalid homogeneous final row.")

    rotation = matrix[:3, :3]
    if not np.allclose(
        rotation.T @ rotation,
        np.eye(3),
        atol=_RIGID_TRANSFORM_ATOL,
        rtol=0.0,
    ):
        raise RuntimeError(f"{name} rotation is not orthonormal.")
    determinant = float(np.linalg.det(rotation))
    if not np.isfinite(determinant) or abs(determinant - 1.0) > _RIGID_TRANSFORM_ATOL:
        raise RuntimeError(
            f"{name} rotation determinant must be +1; got {determinant:.9f}."
        )
    if use_shared_rotation_check:
        assert_valid_rotation(name, matrix)
    return matrix


def _bounds_z_axis_is_world_aligned(T_world_bounds):
    z_axis = np.asarray(T_world_bounds, dtype=float)[:3, 2]
    return bool(
        np.linalg.norm(z_axis[:2]) <= _BOUNDS_Z_ALIGNMENT_ATOL
        and abs(abs(float(z_axis[2])) - 1.0) <= _BOUNDS_Z_ALIGNMENT_ATOL
    )


def _marker_pose_transform(marker):
    pose = getattr(marker, "pose", None)
    position = getattr(pose, "position", None)
    orientation = getattr(pose, "orientation", None)
    if position is None or orientation is None:
        raise RuntimeError("Bounds marker pose is incomplete.")

    translation = np.array(
        [
            float(position.x),
            float(position.y),
            float(position.z),
        ],
        dtype=float,
    )
    quaternion = np.array(
        [
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        ],
        dtype=float,
    )
    if not np.all(np.isfinite(translation)) or not np.all(np.isfinite(quaternion)):
        raise RuntimeError("Bounds marker pose contains NaN or Inf.")
    quaternion_norm = float(np.linalg.norm(quaternion))
    if quaternion_norm <= CLEARANCE_NUMERIC_EPSILON_M:
        raise RuntimeError("Bounds marker quaternion has zero norm.")

    transform = np.eye(4)
    transform[:3, :3] = tf_transformations.quaternion_matrix(
        quaternion / quaternion_norm
    )[:3, :3]
    transform[:3, 3] = translation
    return _validated_rigid_transform("bounds marker pose", transform)


def _marker_to_clearance_obstacle(marker, T_world_marker_frame=None):
    scale = marker.scale
    scale_xyz = np.array(
        [
            float(getattr(scale, "x", 0.0)),
            float(getattr(scale, "y", 0.0)),
            float(getattr(scale, "z", 0.0)),
        ],
        dtype=float,
    )
    if not np.all(np.isfinite(scale_xyz)) or np.any(scale_xyz <= 0.0):
        return None

    try:
        T_marker_frame_bounds = _marker_pose_transform(marker)
        if T_world_marker_frame is None:
            T_world_bounds = T_marker_frame_bounds
        else:
            T_world_marker_frame = _validated_rigid_transform(
                "scene bounds frame TF",
                T_world_marker_frame,
                use_shared_rotation_check=True,
            )
            T_world_bounds = T_world_marker_frame @ T_marker_frame_bounds
        T_world_bounds = _validated_rigid_transform(
            "world bounds transform",
            T_world_bounds,
            use_shared_rotation_check=True,
        )
    except (RuntimeError, TypeError, ValueError):
        return None

    if not _bounds_z_axis_is_world_aligned(T_world_bounds):
        return None

    half_extents = 0.5 * scale_xyz
    center = T_world_bounds[:3, 3].copy()
    radius_xy = float(np.hypot(half_extents[0], half_extents[1]))
    half_height = float(half_extents[2])

    return SceneClearanceObstacle(
        name=_marker_obstacle_name(marker),
        center=center,
        radius_xy=radius_xy,
        half_height=half_height,
        T_world_bounds=T_world_bounds,
        half_extents=half_extents,
    )


def _scene_clearance_obstacles_from_marker_array(
    marker_array,
    selected_object_name,
    node=None,
    world_frame="world",
):
    obstacles = []
    transform_cache = {}
    for marker in getattr(marker_array, "markers", []):
        marker_name = _marker_obstacle_name(marker)
        if _is_target_marker(marker_name, selected_object_name):
            continue

        marker_frame = _marker_frame_id(marker)
        T_world_marker_frame = None
        if marker_frame != world_frame:
            if node is None or not hasattr(node, "tf_buffer"):
                raise RuntimeError(
                    "Scene marker frame requires TF conversion but no "
                    f"node/tf_buffer is available: {marker_frame!r} -> "
                    f"{world_frame!r}."
                )
            if marker_frame not in transform_cache:
                transform_cache[marker_frame] = get_transform_checked(
                    node,
                    marker_frame,
                    world_frame,
                    timeout_sec=0.25,
                )
            T_world_marker_frame = transform_cache[marker_frame]

        obstacle = _marker_to_clearance_obstacle(
            marker,
            T_world_marker_frame=T_world_marker_frame,
        )
        if obstacle is None:
            raise RuntimeError(
                "Could not build clearance obstacle for non-target scene "
                f"marker {marker_name!r}."
            )
        obstacles.append(obstacle)
    return obstacles


def _target_clearance_bounds_from_marker_array(
    marker_array,
    selected_object_name,
    node=None,
    world_frame="world",
):
    matches = [
        marker
        for marker in getattr(marker_array, "markers", [])
        if _is_target_marker(
            _marker_obstacle_name(marker),
            selected_object_name,
        )
    ]
    if len(matches) != 1:
        target_key = _object_name_key(selected_object_name)
        raise RuntimeError(
            "Vertical grasp centering requires exactly one target marker for "
            f"{target_key!r}; found {len(matches)}."
        )

    marker = matches[0]
    marker_name = _marker_obstacle_name(marker)
    marker_frame = _marker_frame_id(marker)
    T_world_marker_frame = None
    if marker_frame != world_frame:
        if node is None or not hasattr(node, "tf_buffer"):
            raise RuntimeError(
                "Target bounds marker frame requires TF conversion but no "
                f"node/tf_buffer is available: {marker_frame!r} -> "
                f"{world_frame!r}."
            )
        T_world_marker_frame = get_transform_checked(
            node,
            marker_frame,
            world_frame,
            timeout_sec=0.25,
        )

    target_bounds = _marker_to_clearance_obstacle(
        marker,
        T_world_marker_frame=T_world_marker_frame,
    )
    if target_bounds is None:
        raise RuntimeError(
            "Could not build usable target bounds for selected object "
            f"{_object_name_key(selected_object_name)!r} from scene marker "
            f"{marker_name!r}."
        )
    return target_bounds


def _wait_for_marker_array(node, topic, timeout_sec):
    if not hasattr(node, "create_subscription"):
        return None

    try:
        from visualization_msgs.msg import MarkerArray
    except Exception as exc:
        _warn(
            node,
            "Cannot import visualization_msgs MarkerArray; clearance bounds "
            f"are unavailable: {exc}",
        )
        return None

    latest = {}
    ready = threading.Event()

    def _callback(msg):
        latest["msg"] = msg
        ready.set()

    callback_group = getattr(node, "cbg", None)
    try:
        subscription = node.create_subscription(
            MarkerArray,
            topic,
            _callback,
            10,
            callback_group=callback_group,
        )
    except TypeError:
        subscription = node.create_subscription(
            MarkerArray,
            topic,
            _callback,
            10,
        )
    except Exception as exc:
        _warn(
            node,
            f"Could not subscribe to {topic}; clearance bounds are "
            f"unavailable: {exc}",
        )
        return None

    try:
        if not ready.wait(timeout=max(float(timeout_sec), 0.0)):
            return None
        return latest.get("msg")
    finally:
        if hasattr(node, "destroy_subscription"):
            try:
                node.destroy_subscription(subscription)
            except Exception:
                pass


def _load_scene_clearance_obstacles(node, selected_object_name):
    marker_array = _wait_for_marker_array(
        node,
        GRASP_SCENE_CLEARANCE_TOPIC,
        GRASP_SCENE_DESCRIPTION_TIMEOUT_SEC,
    )
    if marker_array is None:
        _warn(
            node,
            f"No {GRASP_SCENE_CLEARANCE_TOPIC} MarkerArray received within "
            f"{GRASP_SCENE_DESCRIPTION_TIMEOUT_SEC:.2f}s; scene obstacles "
            "are unavailable for this run.",
        )
        return None

    try:
        obstacles = _scene_clearance_obstacles_from_marker_array(
            marker_array,
            selected_object_name,
            node=node,
        )
    except Exception as exc:
        _warn(
            node,
            f"Could not transform {GRASP_SCENE_CLEARANCE_TOPIC} markers into the planning "
            f"frame; scene obstacles are unavailable: {exc}",
        )
        return None
    print(
        "Approach clearance: loaded "
        f"{len(obstacles)} non-target scene obstacle(s)."
    )
    return obstacles


def _load_vertical_target_clearance_bounds(node, selected_object_name):
    marker_array = _wait_for_marker_array(
        node,
        GRASP_SCENE_CLEARANCE_TOPIC,
        GRASP_SCENE_DESCRIPTION_TIMEOUT_SEC,
    )
    if marker_array is None:
        raise RuntimeError(
            "Vertical grasp centering requires target bounds for "
            f"{selected_object_name!r}, but no {GRASP_SCENE_CLEARANCE_TOPIC} "
            "MarkerArray was received within "
            f"{GRASP_SCENE_DESCRIPTION_TIMEOUT_SEC:.2f}s."
        )

    try:
        target_bounds = _target_clearance_bounds_from_marker_array(
            marker_array,
            selected_object_name,
            node=node,
        )
    except Exception as exc:
        raise RuntimeError(
            "Vertical grasp centering could not load target bounds for "
            f"{selected_object_name!r}: {exc}"
        ) from exc

    print(
        "Vertical grasp target bounds: "
        f"object={selected_object_name!r}, "
        f"center_world={_array_text(target_bounds.center)}, "
        f"half_extents={_array_text(target_bounds.half_extents)}"
    )
    return target_bounds


def _load_apple_scene_bounds_snapshot(node, selected_object_name):
    marker_array = _wait_for_marker_array(
        node,
        GRASP_SCENE_CLEARANCE_TOPIC,
        GRASP_SCENE_DESCRIPTION_TIMEOUT_SEC,
    )
    if marker_array is None:
        raise RuntimeError(
            "Apple scene-bounds canonicalization requires a fresh atomic "
            f"{GRASP_SCENE_CLEARANCE_TOPIC} MarkerArray, but none was received "
            f"within {GRASP_SCENE_DESCRIPTION_TIMEOUT_SEC:.2f}s."
        )

    try:
        target_bounds = _target_clearance_bounds_from_marker_array(
            marker_array,
            selected_object_name,
            node=node,
        )
        obstacles = _scene_clearance_obstacles_from_marker_array(
            marker_array,
            selected_object_name,
            node=node,
        )
    except Exception as exc:
        raise RuntimeError(
            "Apple scene-bounds canonicalization could not use the atomic "
            f"{GRASP_SCENE_CLEARANCE_TOPIC} snapshot: {exc}"
        ) from exc

    print(
        "Apple scene-bounds snapshot: "
        f"target_center_world={_array_text(target_bounds.center)}, "
        f"half_extents={_array_text(target_bounds.half_extents)}, "
        f"non_target_obstacles={len(obstacles)}"
    )
    return target_bounds, obstacles


def _canonicalize_apple_pose_from_scene_bounds(
    T_world_obj_raw,
    target_bounds,
):
    raw = _validated_rigid_transform(
        "FoundationPose Apple world transform",
        T_world_obj_raw,
    )
    T_world_bounds, _ = _precise_bounds_geometry(target_bounds)
    geometry_center_object = _finite_vector(
        "configured Apple geometry center",
        OBJECT_GEOMETRY_BY_NAME["apple"]["center"],
        3,
    )
    bounds_center_world = _finite_vector(
        "Apple scene-bounds center",
        target_bounds.center,
        3,
    )

    foundationpose_geometry_center_world = (
        raw[:3, :3] @ geometry_center_object + raw[:3, 3]
    )
    center_error_m = float(
        np.linalg.norm(
            foundationpose_geometry_center_world - bounds_center_world
        )
    )
    if (
        not np.isfinite(center_error_m)
        or center_error_m > APPLE_SCENE_BOUNDS_CENTER_MAX_ERROR_M
    ):
        raise RuntimeError(
            "Apple FoundationPose and scene-bounds centers disagree: "
            f"error={center_error_m:.4f} m, "
            f"maximum={APPLE_SCENE_BOUNDS_CENTER_MAX_ERROR_M:.4f} m, "
            "refusing to plan."
        )

    bounds_x_axis = T_world_bounds[:3, 0].copy()
    bounds_x_axis[2] = 0.0
    bounds_x_norm = float(np.linalg.norm(bounds_x_axis))
    if not np.isfinite(bounds_x_norm) or bounds_x_norm <= 1e-9:
        raise RuntimeError(
            "Apple scene-bounds X axis has no usable world-XY projection."
        )
    yaw_rad = float(np.arctan2(bounds_x_axis[1], bounds_x_axis[0]))
    cos_yaw = float(np.cos(yaw_rad))
    sin_yaw = float(np.sin(yaw_rad))
    canonical_rotation = np.array(
        [
            [cos_yaw, -sin_yaw, 0.0],
            [sin_yaw, cos_yaw, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    canonical = np.eye(4, dtype=float)
    canonical[:3, :3] = canonical_rotation
    canonical[:3, 3] = (
        bounds_center_world - canonical_rotation @ geometry_center_object
    )
    _validated_rigid_transform("canonical Apple world transform", canonical)

    details = AppleBoundsCanonicalization(
        bounds_center_world=bounds_center_world.copy(),
        foundationpose_geometry_center_world=(
            foundationpose_geometry_center_world.copy()
        ),
        center_error_m=center_error_m,
        canonical_origin_world=canonical[:3, 3].copy(),
    )
    print(
        "Apple scene-bounds canonicalization: "
        f"foundationpose_tilt_to_world_up_deg={object_z_tilt_deg(raw):.2f}, "
        "foundationpose_rotation_ignored=true, "
        f"bounds_center_world={_array_text(bounds_center_world)}, "
        "foundationpose_geometry_center_world="
        f"{_array_text(foundationpose_geometry_center_world)}, "
        f"center_error_m={center_error_m:.4f}, "
        f"canonical_origin_world={_array_text(canonical[:3, 3])}"
    )
    return canonical, details


def _center_vertical_grasp_candidate(
    grasp_pose_6d,
    T_world_grasp,
    target_bounds,
):
    centered_pose = np.asarray(grasp_pose_6d, dtype=float).copy()
    if centered_pose.shape != (6,) or not np.all(np.isfinite(centered_pose)):
        raise RuntimeError("Vertical grasp pose must be a finite 6D vector.")

    centered_transform = np.asarray(T_world_grasp, dtype=float).copy()
    _validated_rigid_transform("vertical world grasp", centered_transform)
    if not np.allclose(
        centered_pose[:3],
        centered_transform[:3, 3],
        atol=1e-6,
        rtol=0.0,
    ):
        raise RuntimeError(
            "Vertical grasp 6D pose and transform translations are inconsistent."
        )

    target_center = np.asarray(target_bounds.center, dtype=float)
    if target_center.shape != (3,) or not np.all(np.isfinite(target_center)):
        raise RuntimeError("Vertical target bounds center must be a finite 3D vector.")

    target_half_height = float(getattr(target_bounds, "half_height", np.nan))
    if not np.isfinite(target_half_height) or target_half_height <= 0.0:
        raise RuntimeError(
            "Vertical target bounds half-height must be finite and positive."
        )
    minimum_clearance_m = float(VERTICAL_MIN_TCP_ABOVE_TARGET_BOTTOM_M)
    if not np.isfinite(minimum_clearance_m) or minimum_clearance_m < 0.0:
        raise RuntimeError(
            "Vertical minimum TCP clearance must be finite and non-negative."
        )

    original_xyz = centered_pose[:3].copy()
    correction_xy = target_center[:2] - centered_pose[:2]
    centered_pose[:2] = target_center[:2]
    bounds_bottom_z = float(target_center[2] - target_half_height)
    minimum_tcp_z = float(bounds_bottom_z + minimum_clearance_m)
    if not np.isfinite(bounds_bottom_z) or not np.isfinite(minimum_tcp_z):
        raise RuntimeError("Vertical target bounds produced a non-finite Z floor.")
    centered_pose[2] = max(float(centered_pose[2]), minimum_tcp_z)
    centered_transform[:3, 3] = centered_pose[:3]
    correction_xyz = centered_pose[:3] - original_xyz
    correction_xyz[:2] = correction_xy
    return centered_pose, centered_transform, correction_xyz


def _xy_distance_point_to_segment(point, segment_start, segment_end):
    point_xy = np.asarray(point, dtype=float)[:2]
    start_xy = np.asarray(segment_start, dtype=float)[:2]
    end_xy = np.asarray(segment_end, dtype=float)[:2]
    segment_xy = end_xy - start_xy
    segment_len_sq = float(segment_xy @ segment_xy)
    if segment_len_sq <= 1e-12:
        return float(np.linalg.norm(point_xy - start_xy))
    t = float(((point_xy - start_xy) @ segment_xy) / segment_len_sq)
    t = float(np.clip(t, 0.0, 1.0))
    closest_xy = start_xy + t * segment_xy
    return float(np.linalg.norm(point_xy - closest_xy))


def _finite_vector(name, value, expected_size):
    vector = np.asarray(value, dtype=float).reshape(-1)
    if vector.size != expected_size:
        raise RuntimeError(f"{name} must contain {expected_size} values.")
    if not np.all(np.isfinite(vector)):
        raise RuntimeError(f"{name} contains NaN or Inf.")
    return vector


def _xy_point_to_rectangle_distance(point, half_extents_xy):
    point_xy = _finite_vector("rectangle query point", point, 2)
    half_extents_xy = _finite_vector(
        "rectangle XY half extents",
        half_extents_xy,
        2,
    )
    if np.any(half_extents_xy <= 0.0):
        raise RuntimeError("Rectangle XY half extents must be positive.")
    outside = np.maximum(np.abs(point_xy) - half_extents_xy, 0.0)
    return float(np.linalg.norm(outside))


def _xy_segment_intersects_rectangle(segment_start, segment_end, half_extents_xy):
    start_xy = _finite_vector("rectangle segment start", segment_start, 2)
    end_xy = _finite_vector("rectangle segment end", segment_end, 2)
    half_extents_xy = _finite_vector(
        "rectangle XY half extents",
        half_extents_xy,
        2,
    )
    if np.any(half_extents_xy <= 0.0):
        raise RuntimeError("Rectangle XY half extents must be positive.")

    direction = end_xy - start_xy
    t_enter = 0.0
    t_exit = 1.0
    for axis in range(2):
        lower = -float(half_extents_xy[axis])
        upper = float(half_extents_xy[axis])
        delta = float(direction[axis])
        start_value = float(start_xy[axis])
        if abs(delta) <= CLEARANCE_NUMERIC_EPSILON_M:
            if (
                start_value < lower - CLEARANCE_NUMERIC_EPSILON_M
                or start_value > upper + CLEARANCE_NUMERIC_EPSILON_M
            ):
                return False
            continue

        axis_enter = (lower - start_value) / delta
        axis_exit = (upper - start_value) / delta
        if axis_enter > axis_exit:
            axis_enter, axis_exit = axis_exit, axis_enter
        t_enter = max(t_enter, axis_enter)
        t_exit = min(t_exit, axis_exit)
        if t_enter > t_exit + CLEARANCE_NUMERIC_EPSILON_M:
            return False
    return True


def _xy_segment_to_rectangle_distance(
    segment_start,
    segment_end,
    half_extents_xy,
):
    start_xy = _finite_vector("rectangle segment start", segment_start, 2)
    end_xy = _finite_vector("rectangle segment end", segment_end, 2)
    half_extents_xy = _finite_vector(
        "rectangle XY half extents",
        half_extents_xy,
        2,
    )
    if np.any(half_extents_xy <= 0.0):
        raise RuntimeError("Rectangle XY half extents must be positive.")
    if _xy_segment_intersects_rectangle(start_xy, end_xy, half_extents_xy):
        return 0.0

    distances = [
        _xy_point_to_rectangle_distance(start_xy, half_extents_xy),
        _xy_point_to_rectangle_distance(end_xy, half_extents_xy),
    ]
    for x_value in (-half_extents_xy[0], half_extents_xy[0]):
        for y_value in (-half_extents_xy[1], half_extents_xy[1]):
            distances.append(
                _xy_distance_point_to_segment(
                    (x_value, y_value),
                    start_xy,
                    end_xy,
                )
            )
    distance = float(min(distances))
    if not np.isfinite(distance):
        raise RuntimeError("Segment-to-rectangle distance is not finite.")
    return distance


def _clip_segment_to_z_slab(segment_start, segment_end, z_min, z_max):
    start = _finite_vector("Z-slab segment start", segment_start, 3)
    end = _finite_vector("Z-slab segment end", segment_end, 3)
    z_limits = _finite_vector("Z-slab limits", (z_min, z_max), 2)
    if z_limits[0] > z_limits[1]:
        raise RuntimeError("Z-slab minimum exceeds its maximum.")

    delta_z = float(end[2] - start[2])
    if abs(delta_z) <= CLEARANCE_NUMERIC_EPSILON_M:
        if (
            start[2] < z_limits[0] - CLEARANCE_NUMERIC_EPSILON_M
            or start[2] > z_limits[1] + CLEARANCE_NUMERIC_EPSILON_M
        ):
            return None
        return 0.0, 1.0

    first = float((z_limits[0] - start[2]) / delta_z)
    second = float((z_limits[1] - start[2]) / delta_z)
    t_enter = max(0.0, min(first, second))
    t_exit = min(1.0, max(first, second))
    if t_enter > t_exit + CLEARANCE_NUMERIC_EPSILON_M:
        return None
    return float(np.clip(t_enter, 0.0, 1.0)), float(
        np.clip(t_exit, 0.0, 1.0)
    )


def _precise_bounds_geometry(obstacle):
    if obstacle.T_world_bounds is None or obstacle.half_extents is None:
        raise RuntimeError(
            f"Scene obstacle {obstacle.name!r} lacks precise bounds geometry."
        )
    try:
        T_world_bounds = _validated_rigid_transform(
            f"scene obstacle {obstacle.name!r} bounds transform",
            obstacle.T_world_bounds,
        )
        half_extents = _finite_vector(
            f"scene obstacle {obstacle.name!r} half extents",
            obstacle.half_extents,
            3,
        )
    except RuntimeError as exc:
        raise RuntimeError(
            f"Scene obstacle {obstacle.name!r} has invalid precise bounds geometry: "
            f"{exc}"
        ) from exc
    if np.any(half_extents <= 0.0):
        raise RuntimeError(
            f"Scene obstacle {obstacle.name!r} precise bounds half extents "
            "must be positive."
        )
    if not _bounds_z_axis_is_world_aligned(T_world_bounds):
        raise RuntimeError(
            f"Scene obstacle {obstacle.name!r} precise bounds local Z axis is "
            "not parallel to world Z."
        )
    center = _finite_vector(
        f"scene obstacle {obstacle.name!r} center",
        obstacle.center,
        3,
    )
    if not np.allclose(
        center,
        T_world_bounds[:3, 3],
        atol=_RIGID_TRANSFORM_ATOL,
        rtol=0.0,
    ):
        raise RuntimeError(
            f"Scene obstacle {obstacle.name!r} center disagrees with its precise "
            "bounds transform."
        )
    return T_world_bounds, half_extents


def _approach_clearance_violation(plan, obstacles, clearance_config):
    pregrasp_xyz = _finite_vector(
        "pregrasp pose XYZ",
        np.asarray(plan.pre_grasp_pose_6d, dtype=float)[:3],
        3,
    )
    grasp_xyz = _finite_vector(
        "grasp pose XYZ",
        np.asarray(plan.grasp_pose_6d, dtype=float)[:3],
        3,
    )
    clearance_values = _finite_vector(
        "approach clearance parameters",
        (
            clearance_config.corridor_radius_m,
            clearance_config.clearance_margin_m,
            clearance_config.vertical_margin_m,
        ),
        3,
    )
    if np.any(clearance_values < 0.0):
        raise RuntimeError("Approach clearance parameters must be non-negative.")
    required_clearance = float(clearance_values[0] + clearance_values[1])
    vertical_margin = float(clearance_values[2])

    for obstacle in obstacles:
        T_world_bounds, half_extents = _precise_bounds_geometry(obstacle)
        rotation = T_world_bounds[:3, :3]
        translation = T_world_bounds[:3, 3]
        pregrasp_local = rotation.T @ (pregrasp_xyz - translation)
        grasp_local = rotation.T @ (grasp_xyz - translation)

        bounds_local_z_min = float(-half_extents[2] - vertical_margin)
        bounds_local_z_max = float(half_extents[2] + vertical_margin)
        overlap = _clip_segment_to_z_slab(
            pregrasp_local,
            grasp_local,
            bounds_local_z_min,
            bounds_local_z_max,
        )
        if overlap is None:
            continue

        t_enter, t_exit = overlap
        local_delta = grasp_local - pregrasp_local
        overlap_start = pregrasp_local + t_enter * local_delta
        overlap_end = pregrasp_local + t_exit * local_delta
        box_xy_distance = _xy_segment_to_rectangle_distance(
            overlap_start[:2],
            overlap_end[:2],
            half_extents[:2],
        )
        clearance = float(box_xy_distance - required_clearance)
        if box_xy_distance <= required_clearance + CLEARANCE_NUMERIC_EPSILON_M:
            return ApproachClearanceViolation(
                obstacle=obstacle,
                box_xy_distance_m=box_xy_distance,
                required_clearance_m=required_clearance,
                clearance_m=clearance,
                bounds_local_z_min=bounds_local_z_min,
                bounds_local_z_max=bounds_local_z_max,
                overlap_t_enter=t_enter,
                overlap_t_exit=t_exit,
            )
    return None


def _filter_results_by_approach_clearance(
    results,
    obstacles,
    selected_object_name,
):
    grasp_profile = get_grasp_profile(selected_object_name)
    clearance_config = _approach_clearance_config(grasp_profile)
    if clearance_config is None or not clearance_config.enabled:
        return list(results)
    if obstacles is None:
        if clearance_config.require_scene:
            raise RuntimeError(
                f"{grasp_profile} approach clearance requires "
                f"{GRASP_SCENE_CLEARANCE_TOPIC}, but no usable scene obstacle list was "
                "received."
            )
        return list(results)

    kept = []
    rejected = 0
    for result in results:
        violation = _approach_clearance_violation(
            result.plan,
            obstacles,
            clearance_config,
        )
        if violation is None:
            kept.append(result)
            continue

        rejected += 1
        obstacle = violation.obstacle
        print(
            f"Rejecting {grasp_profile} candidate {result.candidate_index}: "
            f"approach corridor intersects non-target object "
            f"{obstacle.name!r}; "
            "method=exact_bounds_box_2p5d, "
            f"box_xy_distance={violation.box_xy_distance_m:.4f} m, "
            f"required={violation.required_clearance_m:.4f} m, "
            f"clearance={violation.clearance_m:.4f} m, "
            "bounds_local_z=["
            f"{violation.bounds_local_z_min:.4f}, "
            f"{violation.bounds_local_z_max:.4f}], "
            "overlap_t=["
            f"{violation.overlap_t_enter:.4f}, "
            f"{violation.overlap_t_exit:.4f}], "
            f"half_extents={_array_text(obstacle.half_extents)}"
        )

    print(
        f"{grasp_profile} approach clearance: "
        "method=exact_bounds_box_2p5d, "
        f"checked={len(results)}, rejected={rejected}, remaining={len(kept)}"
    )
    if results and not kept:
        candidate_label = (
            "side-grasp" if grasp_profile == "side" else f"{grasp_profile} grasp"
        )
        raise RuntimeError(
            f"No {candidate_label} candidate clears the approach corridor around "
            "nearby non-target scene objects."
        )

    return kept



def _array_text(value, precision=4):
    return np.array2string(np.asarray(value, dtype=float), precision=precision)


def _warn(node, message):
    if hasattr(node, "get_logger"):
        node.get_logger().warn(message)
    else:
        print(f"[WARN] {message}")


def _info(node, message):
    if hasattr(node, "get_logger"):
        node.get_logger().info(message)
    else:
        print(message)


def _log_object_pose_canonicalization(
    node,
    selected_object_name,
    T_world_obj_raw,
    T_world_obj,
):
    raw_z_axis = object_local_z_axis_world(T_world_obj_raw)
    canonical_z_axis = object_local_z_axis_world(T_world_obj)
    tilt_deg = object_z_tilt_deg(T_world_obj_raw)

    print_pose_summary("raw object pose world/base", T_world_obj_raw)
    print_pose_summary("canonical object pose world/base", T_world_obj)
    print(
        "raw object local z axis in world/base "
        f"{_array_text(raw_z_axis)}; tilt_to_world_up_deg={tilt_deg:.2f}"
    )
    print(
        "canonical object local z axis in world/base "
        f"{_array_text(canonical_z_axis)}"
    )

    if (
        should_canonicalize_tabletop_object_pose(selected_object_name)
        and tilt_deg > GRASP_TABLETOP_CANONICAL_TILT_WARN_DEG
    ):
        _warn(
            node,
            "Detected can pose is tilted; using upright canonical pose for "
            "grasp planning.",
        )


def _side_world_z_min(T_world_obj):
    return float(T_world_obj[2, 3] + SIDE_GRASP_MIN_WORLD_Z_ABOVE_OBJECT_M)


def _passes_side_world_z_filter(
    plan,
    T_world_obj,
    selected_object_name,
    candidate_index,
):
    if get_grasp_profile(selected_object_name) != "side":
        return True

    min_world_z = _side_world_z_min(T_world_obj)
    final_z = float(plan.grasp_pose_6d[2])
    pregrasp_z = float(plan.pre_grasp_pose_6d[2])
    if final_z >= min_world_z and pregrasp_z >= min_world_z:
        return True

    print(
        f"Rejecting side-grasp candidate {candidate_index}: "
        f"final_world_z={final_z:.4f}, pregrasp_world_z={pregrasp_z:.4f}, "
        f"minimum_allowed_world_z={min_world_z:.4f}"
    )
    return False


def _log_selected_world_z_ranges(results):
    if not results:
        return

    final_z = np.array(
        [float(result.plan.grasp_pose_6d[2]) for result in results],
        dtype=float,
    )
    pregrasp_z = np.array(
        [float(result.plan.pre_grasp_pose_6d[2]) for result in results],
        dtype=float,
    )
    print(
        "Selected grasp candidates world z range: "
        f"final=min={final_z.min():.4f}, max={final_z.max():.4f}, "
        f"mean={final_z.mean():.4f}; "
        f"pregrasp=min={pregrasp_z.min():.4f}, max={pregrasp_z.max():.4f}, "
        f"mean={pregrasp_z.mean():.4f}"
    )


def plan_pick_place_candidates_from_perception(
    node,
    start_pose_6d,
    object_cam_pose_path=FOUNDATIONPOSE_OBJECT_CAMERA_POSE_JSON,
    selected_object_path=SELECTED_OBJECT_PATH,
    camera_frame=CAMERA_FRAME,
    canonicalize_tabletop=True,
    sorting_rgb_path=None,
    sorting_depth_path=None,
    classification_mode=None,
):
    selected_object_name = get_selected_object_info(selected_object_path)
    T_world_obj_raw = estimate_object_world_pose(
        node,
        object_cam_pose_path,
        camera_frame,
    )
    apple_canonicalization = None
    preloaded_obstacles = None
    if selected_object_name == "apple" and canonicalize_tabletop:
        apple_target_bounds, preloaded_obstacles = (
            _load_apple_scene_bounds_snapshot(node, selected_object_name)
        )
        T_world_obj, apple_canonicalization = (
            _canonicalize_apple_pose_from_scene_bounds(
                T_world_obj_raw,
                apple_target_bounds,
            )
        )
    else:
        T_world_obj = (
            canonicalize_tabletop_object_pose(
                T_world_obj_raw,
                selected_object_name,
            )
            if canonicalize_tabletop
            else T_world_obj_raw.copy()
        )
    _log_object_pose_canonicalization(
        node,
        selected_object_name,
        T_world_obj_raw,
        T_world_obj,
    )
    grasp_profile = get_grasp_profile(selected_object_name)
    grasp_z_offset = get_grasp_z_offset(selected_object_name)
    grasp_candidates = select_grasp_pose_candidates_6d(
        T_world_obj,
        selected_object_path,
    )
    vertical_target_bounds = (
        _load_vertical_target_clearance_bounds(node, selected_object_name)
        if grasp_profile == "vertical"
        else None
    )
    clearance_config = _approach_clearance_config(grasp_profile)
    if clearance_config is not None and clearance_config.enabled:
        obstacles = (
            preloaded_obstacles
            if preloaded_obstacles is not None
            else _load_scene_clearance_obstacles(node, selected_object_name)
        )
    else:
        obstacles = []
    drop_target_kwargs = {}
    if sorting_rgb_path is not None:
        drop_target_kwargs["rgb_path"] = sorting_rgb_path
    if sorting_depth_path is not None:
        drop_target_kwargs["depth_path"] = sorting_depth_path
    drop_target = resolve_drop_target(
        selected_object_path,
        node=node,
        camera_frame=camera_frame,
        classification_mode=classification_mode,
        **drop_target_kwargs,
    )
    _info(
        node,
        "DROP_TARGET_SELECTED: "
        f"source={drop_target.source}, "
        f"category={drop_target.category}, "
        f"bin={drop_target.bin_name}, "
        f"position={_array_text(drop_target.position)}",
    )
    results = []
    for candidate_index, (grasp_pose_6d, T_world_grasp) in enumerate(
        grasp_candidates,
        start=1,
    ):
        vertical_original_xyz = None
        vertical_grasp_correction_xyz = None
        vertical_bounds_bottom_z = None
        vertical_minimum_tcp_z = None
        if vertical_target_bounds is not None:
            vertical_original_xyz = np.asarray(grasp_pose_6d, dtype=float)[:3].copy()
            (
                grasp_pose_6d,
                T_world_grasp,
                vertical_grasp_correction_xyz,
            ) = _center_vertical_grasp_candidate(
                grasp_pose_6d,
                T_world_grasp,
                vertical_target_bounds,
            )
            vertical_bounds_bottom_z = float(
                vertical_target_bounds.center[2]
                - vertical_target_bounds.half_height
            )
            vertical_minimum_tcp_z = float(
                vertical_bounds_bottom_z
                + VERTICAL_MIN_TCP_ABOVE_TARGET_BOTTOM_M
            )
            print(
                "Vertical grasp centering: "
                f"object={selected_object_name!r}, "
                f"original_xyz={_array_text(vertical_original_xyz)}, "
                "target_bounds_center="
                f"{_array_text(vertical_target_bounds.center)}, "
                f"bounds_bottom_z={vertical_bounds_bottom_z:.4f}, "
                f"minimum_tcp_z={vertical_minimum_tcp_z:.4f}, "
                "correction_xyz="
                f"{_array_text(vertical_grasp_correction_xyz)}, "
                f"centered_xyz={_array_text(grasp_pose_6d[:3])}"
            )
        # ``select_grasp_pose_candidates_6d`` applies the configured vertical
        # offset in world/base Z. Recover the raw library transform here so the
        # grasp-pose debug log can distinguish library geometry from that offset.
        T_world_library_grasp = T_world_grasp.copy()
        vertical_floor_correction_z = (
            float(vertical_grasp_correction_xyz[2])
            if vertical_grasp_correction_xyz is not None
            else 0.0
        )
        T_world_library_grasp[:3, 3] -= np.array(
            [0.0, 0.0, grasp_z_offset + vertical_floor_correction_z],
            dtype=float,
        )
        T_obj_library_grasp = np.linalg.inv(T_world_obj) @ T_world_library_grasp
        pear_close_metrics = None
        if selected_object_name == "pear":
            pear_close_metrics = pear_close_position_metrics(T_obj_library_grasp)
            print(
                "Pear close readiness: "
                f"candidate={candidate_index}, "
                f"alignment_error_deg={pear_close_metrics['alignment_error_deg']:.3f}, "
                f"projected_width_m={pear_close_metrics['projected_width_m']:.5f}, "
                f"opening_margin_m={pear_close_metrics['opening_margin_m']:.5f}, "
                f"expected_position_rad={pear_close_metrics['expected_position_rad']:.3f}, "
                f"minimum_position_rad={pear_close_metrics['minimum_position_rad']:.3f}"
            )
        approach_direction_world = T_world_grasp[:3, :3] @ np.array(
            [0.0, 0.0, 1.0],
            dtype=float,
        )
        approach_direction_world /= np.linalg.norm(approach_direction_world)
        approach_direction_object = (
            T_world_obj[:3, :3].T @ approach_direction_world
        )
        geometry_center_object = get_side_grasp_geometry_center(
            selected_object_name
        )
        plan = plan_safe_pick_place_steps(
            start_pose_6d=start_pose_6d,
            grasp_pose_6d=grasp_pose_6d,
            drop_pose_6d=build_drop_pose_6d(
                grasp_pose_6d,
                drop_target.position,
            ),
            direct_to_grasp=grasp_profile == "side",
            debug_info={
                "object_name": selected_object_name,
                "T_world_obj_raw": T_world_obj_raw.copy(),
                "T_world_obj": T_world_obj.copy(),
                "raw_object_z_axis_world": object_local_z_axis_world(
                    T_world_obj_raw
                ),
                "canonical_object_z_axis_world": object_local_z_axis_world(
                    T_world_obj
                ),
                "grasp_library_z": float(T_obj_library_grasp[2, 3]),
                "side_grasp_z_offset": float(grasp_z_offset),
                "drop_target": drop_target,
                "grasp_profile": grasp_profile,
                **(
                    {}
                    if pear_close_metrics is None
                    else {
                        "pear_projected_width_m": pear_close_metrics[
                            "projected_width_m"
                        ],
                        "pear_expected_close_position_rad": pear_close_metrics[
                            "expected_position_rad"
                        ],
                        "pear_minimum_close_position_rad": pear_close_metrics[
                            "minimum_position_rad"
                        ],
                    }
                ),
                "side_grasp_geometry_center_object": geometry_center_object,
                "approach_direction_world": approach_direction_world,
                "approach_direction_object": approach_direction_object,
                "vertical_grasp_original_xy": (
                    None
                    if vertical_original_xyz is None
                    else vertical_original_xyz[:2].copy()
                ),
                "vertical_target_bounds_xy": (
                    None
                    if vertical_target_bounds is None
                    else vertical_target_bounds.center[:2].copy()
                ),
                "vertical_grasp_correction_xy": (
                    None
                    if vertical_grasp_correction_xyz is None
                    else vertical_grasp_correction_xyz[:2].copy()
                ),
                "vertical_grasp_original_xyz": (
                    None
                    if vertical_original_xyz is None
                    else vertical_original_xyz.copy()
                ),
                "vertical_target_bounds_center": (
                    None
                    if vertical_target_bounds is None
                    else vertical_target_bounds.center.copy()
                ),
                "vertical_bounds_bottom_z": vertical_bounds_bottom_z,
                "vertical_minimum_tcp_z": vertical_minimum_tcp_z,
                "vertical_grasp_correction_xyz": (
                    None
                    if vertical_grasp_correction_xyz is None
                    else vertical_grasp_correction_xyz.copy()
                ),
                **(
                    {}
                    if apple_canonicalization is None
                    else {
                        "apple_pose_source": "scene_description",
                        "apple_scene_bounds_center_world": (
                            apple_canonicalization.bounds_center_world.copy()
                        ),
                        "apple_foundationpose_geometry_center_world": (
                            apple_canonicalization
                            .foundationpose_geometry_center_world
                            .copy()
                        ),
                        "apple_center_error_m": apple_canonicalization.center_error_m,
                        "apple_canonical_origin_world": (
                            apple_canonicalization.canonical_origin_world.copy()
                        ),
                    }
                ),
            },
        )
        print(
            f"Planning grasp candidate {candidate_index}/{len(grasp_candidates)} "
            f"for object {selected_object_name!r}: "
            f"final_grasp_world {pose_text(plan.grasp_pose_6d)}; "
            f"pregrasp_world {pose_text(plan.pre_grasp_pose_6d)}"
        )
        if not _passes_side_world_z_filter(
            plan,
            T_world_obj,
            selected_object_name,
            candidate_index,
        ):
            continue
        results.append(
            PickPlacePlanningResult(
                plan=plan,
                T_world_obj=T_world_obj,
                T_world_grasp=T_world_grasp,
                grasp_pose_6d=grasp_pose_6d,
                candidate_index=candidate_index,
                candidate_count=len(grasp_candidates),
            )
        )
    if not results:
        raise RuntimeError(
            "No grasp candidate passed the world-frame final/pregrasp z "
            "sanity filter."
        )
    results = _filter_results_by_approach_clearance(
        results,
        obstacles,
        selected_object_name,
    )
    for result in results:
        result.candidate_count = len(results)
    _log_selected_world_z_ranges(results)
    return results


def plan_pick_place_from_perception(
    node,
    start_pose_6d,
    object_cam_pose_path=FOUNDATIONPOSE_OBJECT_CAMERA_POSE_JSON,
    selected_object_path=SELECTED_OBJECT_PATH,
    camera_frame=CAMERA_FRAME,
    canonicalize_tabletop=True,
    sorting_rgb_path=None,
    sorting_depth_path=None,
    classification_mode=None,
):
    """Deprecated compatibility wrapper returning the first plan."""
    return plan_pick_place_candidates_from_perception(
        node=node,
        start_pose_6d=start_pose_6d,
        object_cam_pose_path=object_cam_pose_path,
        selected_object_path=selected_object_path,
        camera_frame=camera_frame,
        canonicalize_tabletop=canonicalize_tabletop,
        sorting_rgb_path=sorting_rgb_path,
        sorting_depth_path=sorting_depth_path,
        classification_mode=classification_mode,
    )[0]

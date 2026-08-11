from dataclasses import dataclass
import hashlib
import os
from pathlib import Path
import re
import threading
import time

import numpy as np
import tf_transformations
from sim_pick_place.utils.helpers import _transform_matrix_to_pose6d

from my_course_pkg.grasp.config import (
    CAMERA_FRAME,
    FOUNDATIONPOSE_OBJECT_CAMERA_POSE_JSON,
    GRASP_EXECUTION_MODE,
    GRASP_DEBUG_STOP_AFTER_CLOSE,
    GRASP_DEBUG_STOP_AFTER_LIFT,
    GRASP_DEBUG_STOP_AT_GRASP,
    GRASP_DEBUG_STOP_AT_PREGRASP,
    GRASP_DEBUG_STOP_BEFORE_RELEASE,
    GRASP_LIFT_HEIGHT,
    GRASP_PLACE_BASE_EXCLUSION_ENABLED,
    GRASP_PLACE_BASE_EXCLUSION_X_MAX,
    GRASP_PLACE_BASE_EXCLUSION_X_MIN,
    GRASP_PLACE_BASE_EXCLUSION_Y_MAX,
    GRASP_PLACE_BASE_EXCLUSION_Y_MIN,
    GRASP_PLACE_EDGE_MARGIN_M,
    GRASP_PLACE_ENABLED,
    GRASP_PLACE_GRID_STEP_M,
    GRASP_PLACE_OBJECT_CLEARANCE_M,
    GRASP_PLACE_PREFERRED_X,
    GRASP_PLACE_PREFERRED_Y,
    GRASP_PLACE_REQUIRE_SCENE,
    GRASP_PLACE_USE_SCENE,
    GRASP_PLACE_X_MAX,
    GRASP_PLACE_X_MIN,
    GRASP_PLACE_Y_MAX,
    GRASP_PLACE_Y_MIN,
    GRASP_SCENE_CLEARANCE_TOPIC,
    GRASP_SCENE_DESCRIPTION_TIMEOUT_SEC,
    GRASP_TABLETOP_CANONICAL_TILT_WARN_DEG,
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
    GRIPPER_OPEN_POSITION,
    VERTICAL_MIN_TCP_ABOVE_TARGET_BOTTOM_M,
    read_tuna_config,
)
from my_course_pkg.grasp.tuna_calibration import load_tuna_gripper_calibration
from my_course_pkg.grasp.tuna_errors import TunaErrorCode, TunaGraspError
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
    build_tuna_preclamp_plan,
    build_drop_pose_6d,
    plan_safe_pick_place_steps,
)
from my_course_pkg.grasp.tuna_roll_grasp import (
    FrozenPivotContract,
    TunaBoundsSample,
    build_contact_support_pose,
    build_pregrasp_pose,
    evaluate_initial_straddle,
    generate_radial_directions,
    generate_roll_waypoints,
    interpolate_gripper_geometry,
    required_width,
    support_hull_clearance_lower_bound,
    validate_tuna_bounds_samples,
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
class TunaPlanningCandidate:
    radial_index: int
    radial_direction: np.ndarray
    contact_height_m: float
    pitch_deg: float
    T_world_tcp_contact: np.ndarray
    T_world_tcp_pregrasp: np.ndarray
    clearance_lower_bound_m: float
    reachability_score: float


@dataclass(frozen=True)
class TunaPerceptionProvenance:
    cache_invalidation_verified: bool
    invalidated_wall_time: float | None
    scene_reset_wall_time: float | None
    files: tuple[dict, ...]


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
class SafePlaceSelection:
    xy: np.ndarray
    min_clearance_m: float
    obstacle_count: int
    score: float


def _array_text(value, precision=4):
    return np.array2string(np.asarray(value, dtype=float), precision=precision)


def _warn(node, message):
    if hasattr(node, "get_logger"):
        node.get_logger().warn(message)
    else:
        print(f"[WARN] {message}")


def _normalize_tuna_object_name(value):
    """Normalize spelling only; deliberately do not strip dataset prefixes."""
    key = str(value or "").strip().lower().replace("-", "_").replace(" ", "_")
    return re.sub(r"_+", "_", key)


def _legacy_debug_flags_enabled():
    return tuple(
        name
        for name, enabled in (
            ("GRASP_DEBUG_STOP_AT_PREGRASP", GRASP_DEBUG_STOP_AT_PREGRASP),
            ("GRASP_DEBUG_STOP_AT_GRASP", GRASP_DEBUG_STOP_AT_GRASP),
            ("GRASP_DEBUG_STOP_AFTER_CLOSE", GRASP_DEBUG_STOP_AFTER_CLOSE),
            ("GRASP_DEBUG_STOP_AFTER_LIFT", GRASP_DEBUG_STOP_AFTER_LIFT),
            ("GRASP_DEBUG_STOP_BEFORE_RELEASE", GRASP_DEBUG_STOP_BEFORE_RELEASE),
        )
        if enabled
    )


def _file_provenance(path):
    candidate = Path(path).expanduser()
    if not candidate.is_file():
        return {
            "path": str(candidate),
            "exists": False,
            "mtime_wall_time": None,
            "sha256": None,
        }
    payload = candidate.read_bytes()
    return {
        "path": str(candidate.resolve()),
        "exists": True,
        "mtime_wall_time": float(candidate.stat().st_mtime),
        "sha256": hashlib.sha256(payload).hexdigest(),
    }


def _validate_tuna_perception_cache(
    node,
    *,
    selected_object_path,
    object_cam_pose_path,
    require_fresh_scene,
):
    invalidated = getattr(node, "tuna_perception_cache_invalidated_wall_time", None)
    reset = getattr(node, "tuna_scene_reset_wall_time", None)
    invalidated = None if invalidated is None else float(invalidated)
    reset = None if reset is None else float(reset)
    if invalidated is not None and not np.isfinite(invalidated):
        invalidated = None
    if reset is not None and not np.isfinite(reset):
        reset = None
    files = tuple(
        _file_provenance(path)
        for path in (selected_object_path, object_cam_pose_path)
    )
    verified = bool(
        invalidated is not None
        and reset is not None
        and invalidated >= reset
        and all(
            item["exists"]
            and item["mtime_wall_time"] is not None
            and item["mtime_wall_time"] > invalidated
            for item in files
        )
    )
    if require_fresh_scene and not verified:
        raise TunaGraspError(
            TunaErrorCode.BOUNDS,
            "perception_cache",
            {
                "object_name": "tuna_fish_can",
                "reason": "fresh_scene_cache_invalidation_unverified",
                "has_invalidation_boundary": invalidated is not None,
                "has_scene_reset_boundary": reset is not None,
                "files_exist": [bool(item["exists"]) for item in files],
            },
        )
    return TunaPerceptionProvenance(
        cache_invalidation_verified=verified,
        invalidated_wall_time=invalidated,
        scene_reset_wall_time=reset,
        files=files,
    )


def _stamp_seconds(stamp):
    if stamp is None:
        raise ValueError("Tuna bounds marker has no source timestamp.")
    value = float(getattr(stamp, "sec", 0)) + 1e-9 * float(
        getattr(stamp, "nanosec", 0)
    )
    if not np.isfinite(value):
        raise ValueError("Tuna bounds marker source timestamp is non-finite.")
    return value


def _tuna_bounds_sample_from_marker(marker, received_monotonic):
    if _normalize_tuna_object_name(_marker_obstacle_name(marker)) != "tuna_fish_can":
        raise ValueError("Tuna bounds marker identity is not exact tuna_fish_can.")
    if _marker_frame_id(marker) != "world":
        raise ValueError("Tuna bounds marker must already be in the world frame.")
    transform = _marker_pose_transform(marker)
    if not _bounds_z_axis_is_world_aligned(transform):
        raise ValueError("Tuna bounds marker may contain yaw but not roll or pitch.")
    scale = np.array(
        [float(marker.scale.x), float(marker.scale.y), float(marker.scale.z)],
        dtype=float,
    )
    return TunaBoundsSample(
        object_name="tuna_fish_can",
        frame_id="world",
        source_timestamp=_stamp_seconds(getattr(marker.header, "stamp", None)),
        received_monotonic=float(received_monotonic),
        center=transform[:3, 3],
        size=scale,
    )


def _collect_tuna_bounds_samples(node, required_count, reception_boundary):
    collector = getattr(node, "collect_tuna_bounds_samples", None)
    if callable(collector):
        return tuple(
            collector(
                required_count=int(required_count),
                reception_boundary=float(reception_boundary),
            )
        )
    if not hasattr(node, "create_subscription"):
        raise TunaGraspError(
            TunaErrorCode.BOUNDS,
            "initial_bounds",
            {
                "object_name": "tuna_fish_can",
                "reason": "bounds_collector_unavailable",
            },
        )
    try:
        from visualization_msgs.msg import MarkerArray
    except Exception as exc:
        raise TunaGraspError(
            TunaErrorCode.BOUNDS,
            "initial_bounds",
            {
                "object_name": "tuna_fish_can",
                "reason": "marker_array_import",
            },
        ) from exc

    samples = []
    callback_errors = []
    ready = threading.Event()

    def _callback(message):
        received = time.monotonic()
        if received <= reception_boundary:
            return
        exact = [
            marker
            for marker in getattr(message, "markers", ())
            if _normalize_tuna_object_name(_marker_obstacle_name(marker))
            == "tuna_fish_can"
        ]
        if len(exact) != 1:
            callback_errors.append(f"exact_marker_count:{len(exact)}")
            return
        try:
            sample = _tuna_bounds_sample_from_marker(exact[0], received)
        except (TypeError, ValueError, RuntimeError) as exc:
            callback_errors.append(type(exc).__name__)
            return
        if samples and sample.source_timestamp <= samples[-1].source_timestamp:
            return
        samples.append(sample)
        if len(samples) >= required_count:
            ready.set()

    callback_group = getattr(node, "cbg", None)
    try:
        subscription = node.create_subscription(
            MarkerArray,
            GRASP_SCENE_CLEARANCE_TOPIC,
            _callback,
            10,
            callback_group=callback_group,
        )
    except TypeError:
        subscription = node.create_subscription(
            MarkerArray,
            GRASP_SCENE_CLEARANCE_TOPIC,
            _callback,
            10,
        )
    try:
        ready.wait(GRASP_SCENE_DESCRIPTION_TIMEOUT_SEC)
    finally:
        if hasattr(node, "destroy_subscription"):
            node.destroy_subscription(subscription)
    if len(samples) != required_count:
        raise TunaGraspError(
            TunaErrorCode.BOUNDS,
            "initial_bounds",
            {
                "object_name": "tuna_fish_can",
                "reason": "three_fresh_samples_not_received",
                "received_count": len(samples),
                "callback_errors": callback_errors[:5],
            },
        )
    return tuple(samples)


def _tuna_table_z(node):
    getter = getattr(node, "get_tuna_table_z_m", None)
    value = getter() if callable(getter) else getattr(node, "tuna_table_z_m", None)
    if value is None and hasattr(node, "create_client"):
        from my_course_pkg.grasp.tuna_planning_scene import (
            read_tuna_table_z_from_moveit,
        )

        value = read_tuna_table_z_from_moveit(node)
    if value is None or not np.isfinite(float(value)):
        raise TunaGraspError(
            TunaErrorCode.BOUNDS,
            "planning_scene_table",
            {
                "object_name": "tuna_fish_can",
                "reason": "planning_scene_table_z_unavailable",
            },
        )
    return float(value)


def _default_tuna_calibration_sources():
    root = Path(__file__).resolve().parents[4]
    mesh_root = (
        root
        / "src"
        / "ros2_robotiq_gripper"
        / "robotiq_description"
        / "meshes"
        / "collision"
        / "2f_85"
    )
    names = {
        "robotiq_85_base_link": "robotiq_base.stl",
        "robotiq_85_left_finger_link": "left_finger.stl",
        "robotiq_85_left_finger_tip_link": "left_finger_tip_160_collision.stl",
        "robotiq_85_left_inner_knuckle_link": "left_inner_knuckle.stl",
        "robotiq_85_left_knuckle_link": "left_knuckle.stl",
        "robotiq_85_right_finger_link": "right_finger.stl",
        "robotiq_85_right_finger_tip_link": "right_finger_tip_160_collision.stl",
        "robotiq_85_right_inner_knuckle_link": "right_inner_knuckle.stl",
        "robotiq_85_right_knuckle_link": "right_knuckle.stl",
    }
    return {
        "urdf_path": root
        / "src"
        / "ifl_air_cell_small_ur_orbbec_robotiq_moveit_config"
        / "urdf"
        / "cell_small_ur_orbbec_robotiq2f85.urdf",
        "collision_mesh_paths": {
            link_name: mesh_root / filename for link_name, filename in names.items()
        },
        "mount_transforms_path": root
        / "src"
        / "my_course_pkg"
        / "tools"
        / "tuna_gripper_mount_transforms.json",
    }


def _load_tuna_calibration_for_planning(node):
    loader = getattr(node, "load_tuna_calibration", None)
    if callable(loader):
        return loader()
    sources = getattr(node, "tuna_calibration_sources", None)
    if sources is None:
        sources = _default_tuna_calibration_sources()
    return load_tuna_gripper_calibration(**dict(sources))


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


def _safe_place_grid_values(lower, upper, margin, step):
    lower = float(lower) + float(margin)
    upper = float(upper) - float(margin)
    step = max(float(step), 1e-3)
    if lower > upper:
        raise RuntimeError(
            "Safe-place table bounds are invalid after edge margin: "
            f"lower={lower:.3f}, upper={upper:.3f}."
        )
    values = np.arange(lower, upper + 0.5 * step, step, dtype=float)
    if values.size == 0:
        return np.array([(lower + upper) * 0.5], dtype=float)
    return values


def _place_clearance_to_obstacles(xy, obstacles):
    xy = np.asarray(xy, dtype=float)[:2]
    if not obstacles:
        return float("inf")
    clearances = []
    for obstacle in obstacles:
        obstacle_xy = np.asarray(obstacle.center, dtype=float)[:2]
        center_distance = float(np.linalg.norm(xy - obstacle_xy))
        clearances.append(center_distance - float(obstacle.radius_xy))
    return float(min(clearances))


def _in_base_exclusion_zone(xy, x_min, x_max, y_min, y_max):
    x, y = np.asarray(xy, dtype=float)[:2]
    return (
        float(x_min) <= float(x) <= float(x_max)
        and float(y_min) <= float(y) <= float(y_max)
    )


def _select_safe_place_xy(obstacles, preferred_xy=None):
    obstacles = [] if obstacles is None else list(obstacles)
    preferred_xy = np.asarray(
        (
            [GRASP_PLACE_PREFERRED_X, GRASP_PLACE_PREFERRED_Y]
            if preferred_xy is None
            else preferred_xy
        ),
        dtype=float,
    )[:2]
    x_values = _safe_place_grid_values(
        GRASP_PLACE_X_MIN,
        GRASP_PLACE_X_MAX,
        GRASP_PLACE_EDGE_MARGIN_M,
        GRASP_PLACE_GRID_STEP_M,
    )
    y_values = _safe_place_grid_values(
        GRASP_PLACE_Y_MIN,
        GRASP_PLACE_Y_MAX,
        GRASP_PLACE_EDGE_MARGIN_M,
        GRASP_PLACE_GRID_STEP_M,
    )

    print(
        "[PlaceDebug] "
        f"GRASP_PLACE_ENABLED={int(bool(GRASP_PLACE_ENABLED))}"
    )
    print(
        "[PlaceDebug] checking bounds: "
        f"x=[{GRASP_PLACE_X_MIN:.3f}, {GRASP_PLACE_X_MAX:.3f}], "
        f"y=[{GRASP_PLACE_Y_MIN:.3f}, {GRASP_PLACE_Y_MAX:.3f}], "
        f"edge_margin={GRASP_PLACE_EDGE_MARGIN_M:.3f}, "
        f"grid_step={GRASP_PLACE_GRID_STEP_M:.3f}"
    )
    if GRASP_PLACE_BASE_EXCLUSION_ENABLED:
        print(
            "[PlaceDebug] base_exclusion_zone: "
            f"x=[{GRASP_PLACE_BASE_EXCLUSION_X_MIN:.3f}, "
            f"{GRASP_PLACE_BASE_EXCLUSION_X_MAX:.3f}], "
            f"y=[{GRASP_PLACE_BASE_EXCLUSION_Y_MIN:.3f}, "
            f"{GRASP_PLACE_BASE_EXCLUSION_Y_MAX:.3f}]"
        )
    else:
        print("[PlaceDebug] base_exclusion_zone: disabled")

    best = None
    checked = 0
    rejected_by_base = 0
    rejected_by_clearance = 0
    for x in x_values:
        for y in y_values:
            checked += 1
            xy = np.array([x, y], dtype=float)
            if GRASP_PLACE_BASE_EXCLUSION_ENABLED and _in_base_exclusion_zone(
                xy,
                GRASP_PLACE_BASE_EXCLUSION_X_MIN,
                GRASP_PLACE_BASE_EXCLUSION_X_MAX,
                GRASP_PLACE_BASE_EXCLUSION_Y_MIN,
                GRASP_PLACE_BASE_EXCLUSION_Y_MAX,
            ):
                rejected_by_base += 1
                continue

            min_clearance = _place_clearance_to_obstacles(xy, obstacles)
            if min_clearance < GRASP_PLACE_OBJECT_CLEARANCE_M:
                rejected_by_clearance += 1
                continue

            preferred_distance = float(np.linalg.norm(xy - preferred_xy))
            finite_clearance = (
                min_clearance
                if np.isfinite(min_clearance)
                else GRASP_PLACE_OBJECT_CLEARANCE_M
            )
            score = finite_clearance - 0.25 * preferred_distance
            candidate = SafePlaceSelection(
                xy=xy,
                min_clearance_m=min_clearance,
                obstacle_count=len(obstacles),
                score=float(score),
            )
            if best is None or candidate.score > best.score:
                best = candidate

    if best is None:
        raise RuntimeError(
            "No safe placement point found inside the configured table region. "
            f"checked={checked}, obstacle_count={len(obstacles)}, "
            f"required_clearance={GRASP_PLACE_OBJECT_CLEARANCE_M:.3f} m, "
            f"rejected_by_clearance={rejected_by_clearance}, "
            f"rejected_by_base={rejected_by_base}."
        )

    selected_preferred_distance = float(np.linalg.norm(best.xy - preferred_xy))
    print(
        "[PlaceDebug] checked="
        f"{checked}, rejected_by_clearance={rejected_by_clearance}, "
        f"rejected_by_base={rejected_by_base}, "
        f"preferred_distance={selected_preferred_distance:.4f}"
    )
    print(
        "[PlaceDebug] selected_place_xy="
        f"{_array_text(best.xy)}, clearance={best.min_clearance_m:.4f}, "
        f"score={best.score:.4f}"
    )
    print(
        "Safe placement selected: "
        f"xy={_array_text(best.xy)}, "
        f"min_clearance_m={best.min_clearance_m:.4f}, "
        f"obstacles={best.obstacle_count}, score={best.score:.4f}"
    )
    return best


def _load_obstacles_for_place_and_clearance(node, selected_object_name, grasp_profile):
    needs_scene_for_place = (
        GRASP_EXECUTION_MODE == "safe_place"
        and GRASP_PLACE_ENABLED
        and GRASP_PLACE_USE_SCENE
    )
    clearance_config = _approach_clearance_config(grasp_profile)
    needs_scene_for_approach_clearance = bool(
        clearance_config is not None and clearance_config.enabled
    )
    if not needs_scene_for_place and not needs_scene_for_approach_clearance:
        return None

    obstacles = _load_scene_clearance_obstacles(node, selected_object_name)
    if (
        obstacles is None
        and needs_scene_for_approach_clearance
        and clearance_config.require_scene
    ):
        raise RuntimeError(
            f"{grasp_profile} approach clearance requires "
            f"{GRASP_SCENE_CLEARANCE_TOPIC}, "
            "but no usable scene obstacle list was received. Refusing to plan "
            "the pregrasp-to-grasp approach."
        )
    if obstacles is None and needs_scene_for_place and GRASP_PLACE_REQUIRE_SCENE:
        raise RuntimeError(
            f"Safe placement requires {GRASP_SCENE_CLEARANCE_TOPIC}, but no usable scene "
            "obstacle list was received. Refusing to plan the drop/release "
            "stage."
        )
    return obstacles


def _build_safe_drop_pose_for_grasp(grasp_pose_6d, safe_place_selection):
    if not GRASP_PLACE_ENABLED or safe_place_selection is None:
        return build_drop_pose_6d(grasp_pose_6d)
    drop_position = np.array(
        [
            safe_place_selection.xy[0],
            safe_place_selection.xy[1],
            0.0,
        ],
        dtype=float,
    )
    return build_drop_pose_6d(
        grasp_pose_6d,
        drop_position=drop_position,
        release_z=float(np.asarray(grasp_pose_6d, dtype=float)[2]),
    )


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


def _tuna_generation_plan(start_pose_6d, candidate, debug_info):
    pregrasp_pose = _transform_matrix_to_pose6d(
        np.array(candidate.T_world_tcp_pregrasp, dtype=float, copy=True)
    )
    contact_pose = _transform_matrix_to_pose6d(
        np.array(candidate.T_world_tcp_contact, dtype=float, copy=True)
    )
    debug = dict(debug_info)
    debug.update(
        {
            "object_name": "tuna_fish_can",
            "grasp_profile": "tuna_roll_up",
            "tuna_policy": "roll_up",
            "tuna_non_executable": True,
            "tuna_non_executable_reason": (
                "generation stage uses nominal calibration geometry and emits "
                "no physical commands"
            ),
        }
    )
    return PickPlacePlan(
        start_pose_6d=np.asarray(start_pose_6d, dtype=float).copy(),
        pre_grasp_pose_6d=pregrasp_pose,
        grasp_pose_6d=contact_pose,
        drop_high_pose_6d=contact_pose.copy(),
        drop_pose_6d=contact_pose.copy(),
        steps=[],
        debug_info=debug,
        deferred_tuna_suffix=None,
        plan_revision=0,
    )


def _validate_tuna_candidate_motion(node, candidate_index, poses):
    validator = getattr(node, "validate_tuna_candidate_sequence", None)
    if not callable(validator):
        motion_executor = getattr(node, "motion_executor", None)
        state_reader = getattr(motion_executor, "current_tuna_joint_state", None)
        if not callable(state_reader):
            raise TunaGraspError(
                TunaErrorCode.CONFIGURATION,
                "candidate_moveit_validation",
                {
                    "object_name": "tuna_fish_can",
                    "reason": "full_chain_moveit_validator_unavailable",
                },
            )
        from my_course_pkg.grasp.tuna_finalizer import (
            MoveItPoseValidation,
            RosMoveItValidationBackend,
        )

        backend = getattr(node, "_tuna_candidate_moveit_backend", None)
        if backend is None:
            backend = RosMoveItValidationBackend(node)
            setattr(node, "_tuna_candidate_moveit_backend", backend)
        joint_names, joint_positions = state_reader()
        for pose_index, transform in enumerate(poses):
            pose_6d = _transform_matrix_to_pose6d(
                np.array(transform, dtype=float, copy=True)
            )
            validation = backend.validate_pose(
                pose_6d,
                joint_names,
                joint_positions,
                f"candidate_{candidate_index}:pose_{pose_index}",
            )
            if not isinstance(validation, MoveItPoseValidation) or not validation.success:
                return False, 0.0
            joint_names = validation.joint_names
            joint_positions = validation.joint_positions
        return True, 0.0
    pose_6d_values = tuple(
        _transform_matrix_to_pose6d(np.array(pose, dtype=float, copy=True))
        for pose in poses
    )
    outcome = validator(
        candidate_index=int(candidate_index),
        poses=pose_6d_values,
        target_object_name="tuna_fish_can",
        allowed_contact_links=(
            "robotiq_85_left_finger_tip_link",
            "robotiq_85_right_finger_tip_link",
        ),
    )
    if isinstance(outcome, bool):
        passed = outcome
        score = 0.0
    elif isinstance(outcome, dict):
        passed = bool(outcome.get("passed", False))
        score = float(outcome.get("reachability_score", 0.0))
    else:
        passed = bool(getattr(outcome, "passed", False))
        score = float(getattr(outcome, "reachability_score", 0.0))
    if not np.isfinite(score):
        raise TunaGraspError(
            TunaErrorCode.PLANNING,
            "candidate_moveit_validation",
            {
                "object_name": "tuna_fish_can",
                "reason": "nonfinite_reachability_score",
                "candidate_index": int(candidate_index),
            },
        )
    return passed, score


def _enumerate_tuna_candidates(
    *,
    node,
    config,
    bounds,
    table_z_m,
    calibration,
    geometry,
    require_moveit,
):
    open_geometry = interpolate_gripper_geometry(calibration, GRIPPER_OPEN_POSITION)
    diameter_m = float(2.0 * bounds.radius_m)
    height_m = float(bounds.size[2])
    candidates = []
    rejected = []
    candidate_index = 0
    for radial_index, radial in enumerate(
        generate_radial_directions(config.radial_direction_count)
    ):
        # Reviewed sign convention: positive roll-in is right-handed about
        # world-up x near-side radial and must never be changed per candidate.
        roll_axis = np.cross(np.array([0.0, 0.0, 1.0]), radial)
        for contact_height_m in config.contact_heights_m:
            for pitch_deg in config.tool_z_angles_to_horizontal_deg:
                candidate_index += 1
                projected_width = required_width(height_m, diameter_m, pitch_deg)
                straddle = evaluate_initial_straddle(
                    open_geometry.pad_gap_m,
                    projected_width,
                    config.min_straddle_margin_m,
                )
                if not straddle.passed:
                    rejected.append(
                        {
                            "candidate_index": candidate_index,
                            "reason": "initial_straddle",
                            "residual_m": float(straddle.residual),
                        }
                    )
                    continue
                contact = build_contact_support_pose(
                    center_xy=bounds.center[:2],
                    radius_m=bounds.radius_m,
                    bottom_z_m=bounds.bottom_z_m,
                    radial_direction=radial,
                    contact_height_m=contact_height_m,
                    tool_z_angle_to_horizontal_deg=pitch_deg,
                    T_tcp_lower_pad_contact=geometry.T_tcp_lower_pad_contact,
                )
                pregrasp = build_pregrasp_pose(contact, config.approach_dist_m)
                pivot = FrozenPivotContract(
                    calibration_sha256=calibration.source_sha256,
                    sample_timestamps=(1.0, 2.0, 3.0),
                    qpos_peak_to_peak_rad=0.0,
                    measured_qpos=geometry.measured_qpos,
                    measured_pad_gap_m=geometry.pad_gap_m,
                    T_world_tcp_preclamp=contact,
                    T_tcp_pivot_frozen=geometry.T_tcp_lower_pad_contact,
                    T_world_pivot=contact @ geometry.T_tcp_lower_pad_contact,
                )
                roll_waypoints = generate_roll_waypoints(
                    pivot,
                    roll_axis,
                    total_angle_deg=config.roll_checkpoint_angles_deg[-1],
                    max_microsegment_angle_deg=(
                        config.roll_microsegment_max_angle_deg
                    ),
                )
                pose_sequence = (pregrasp, contact) + tuple(
                    waypoint.T_world_tcp for waypoint in roll_waypoints
                )
                clearance_records = []
                envelope_geometries = tuple(calibration.points) + (geometry,)
                for envelope_geometry in envelope_geometries:
                    envelope_qpos = float(
                        getattr(
                            envelope_geometry,
                            "qpos",
                            getattr(envelope_geometry, "measured_qpos", np.nan),
                        )
                    )
                    envelope_contact = build_contact_support_pose(
                        center_xy=bounds.center[:2],
                        radius_m=bounds.radius_m,
                        bottom_z_m=bounds.bottom_z_m,
                        radial_direction=radial,
                        contact_height_m=contact_height_m,
                        tool_z_angle_to_horizontal_deg=pitch_deg,
                        T_tcp_lower_pad_contact=(
                            envelope_geometry.T_tcp_lower_pad_contact
                        ),
                    )
                    envelope_pregrasp = build_pregrasp_pose(
                        envelope_contact,
                        config.approach_dist_m,
                    )
                    envelope_pivot = FrozenPivotContract(
                        calibration_sha256=calibration.source_sha256,
                        sample_timestamps=(1.0, 2.0, 3.0),
                        qpos_peak_to_peak_rad=0.0,
                        measured_qpos=envelope_qpos,
                        measured_pad_gap_m=float(envelope_geometry.pad_gap_m),
                        T_world_tcp_preclamp=envelope_contact,
                        T_tcp_pivot_frozen=(
                            envelope_geometry.T_tcp_lower_pad_contact
                        ),
                        T_world_pivot=(
                            envelope_contact
                            @ envelope_geometry.T_tcp_lower_pad_contact
                        ),
                    )
                    envelope_roll = generate_roll_waypoints(
                        envelope_pivot,
                        roll_axis,
                        total_angle_deg=config.roll_checkpoint_angles_deg[-1],
                        max_microsegment_angle_deg=(
                            config.roll_microsegment_max_angle_deg
                        ),
                    )
                    envelope_poses = (envelope_pregrasp, envelope_contact) + tuple(
                        waypoint.T_world_tcp for waypoint in envelope_roll
                    )
                    for pose_index, envelope_pose in enumerate(envelope_poses):
                        clearance_records.append(
                            (
                                envelope_qpos,
                                pose_index,
                                support_hull_clearance_lower_bound(
                                    envelope_pose,
                                    envelope_geometry.support_hull_tcp,
                                    table_z_m,
                                    calibration.max_interpolation_error_m,
                                ),
                            )
                        )
                minimum_record = min(clearance_records, key=lambda item: item[2])
                clearance = float(minimum_record[2])
                if clearance < config.min_table_clearance_m:
                    desired_records = [
                        item
                        for item in clearance_records
                        if np.isclose(
                            item[0], geometry.measured_qpos, rtol=0.0, atol=1e-12
                        )
                    ]
                    rejected.append(
                        {
                            "candidate_index": candidate_index,
                            "reason": "support_hull_table_clearance",
                            "clearance_m": clearance,
                            "minimum_qpos": float(minimum_record[0]),
                            "minimum_pose_index": int(minimum_record[1]),
                            "pregrasp_clearance_m": float(desired_records[0][2]),
                            "contact_clearance_m": float(desired_records[1][2]),
                            "roll_minimum_clearance_m": float(
                                min(item[2] for item in desired_records[2:])
                            ),
                        }
                    )
                    continue
                reachability_score = 0.0
                if require_moveit:
                    passed, reachability_score = _validate_tuna_candidate_motion(
                        node,
                        candidate_index,
                        pose_sequence,
                    )
                    if not passed:
                        rejected.append(
                            {
                                "candidate_index": candidate_index,
                                "reason": "moveit_full_chain",
                            }
                        )
                        continue
                candidates.append(
                    TunaPlanningCandidate(
                        radial_index=radial_index,
                        radial_direction=np.array(radial, dtype=float, copy=True),
                        contact_height_m=float(contact_height_m),
                        pitch_deg=float(pitch_deg),
                        T_world_tcp_contact=np.array(contact, copy=True),
                        T_world_tcp_pregrasp=np.array(pregrasp, copy=True),
                        clearance_lower_bound_m=clearance,
                        reachability_score=reachability_score,
                    )
                )
    candidates.sort(
        key=lambda candidate: (
            -candidate.reachability_score,
            -candidate.clearance_lower_bound_m,
            candidate.radial_index,
            candidate.contact_height_m,
            candidate.pitch_deg,
        )
    )
    return tuple(candidates), tuple(rejected), candidate_index


def _plan_tuna_fish_can_candidates(
    *,
    node,
    start_pose_6d,
    T_world_obj_raw,
    selected_object_path,
    object_cam_pose_path,
):
    try:
        config = read_tuna_config()
    except (TypeError, ValueError) as exc:
        raise TunaGraspError(
            TunaErrorCode.CONFIGURATION,
            "config",
            {"object_name": "tuna_fish_can", "reason": type(exc).__name__},
        ) from exc
    legacy_flags = _legacy_debug_flags_enabled()
    if legacy_flags:
        raise TunaGraspError(
            TunaErrorCode.CONFIGURATION,
            "debug_selector",
            {
                "object_name": "tuna_fish_can",
                "reason": "legacy_and_tuna_debug_selectors_conflict",
                "legacy_flags": list(legacy_flags),
            },
        )
    is_generation = config.debug_stop_after == "generation"
    if not is_generation and config.preclamp_position is None:
        raise TunaGraspError(
            TunaErrorCode.CONFIGURATION,
            "preclamp",
            {
                "object_name": "tuna_fish_can",
                "reason": "qualified_preclamp_position_missing",
            },
        )
    provenance = _validate_tuna_perception_cache(
        node,
        selected_object_path=selected_object_path,
        object_cam_pose_path=object_cam_pose_path,
        require_fresh_scene=not is_generation,
    )
    table_z_m = _tuna_table_z(node)
    reception_boundary = time.monotonic()
    source_boundary = float(getattr(node, "tuna_bounds_source_boundary", 0.0))
    samples = _collect_tuna_bounds_samples(node, 3, reception_boundary)
    try:
        bounds = validate_tuna_bounds_samples(
            samples,
            required_count=3,
            source_boundary=source_boundary,
            reception_boundary=reception_boundary,
            stability_tolerance_m=config.initial_bounds_stability_tolerance_m,
            table_z_m=table_z_m,
            table_consistency_tolerance_m=(
                config.initial_bounds_stability_tolerance_m
            ),
        )
    except (TypeError, ValueError) as exc:
        raise TunaGraspError(
            TunaErrorCode.BOUNDS,
            "initial_bounds",
            {
                "object_name": "tuna_fish_can",
                "reason": str(exc),
            },
        ) from exc
    calibration, artifact = _load_tuna_calibration_for_planning(node)
    if is_generation:
        q_min = calibration.points[0].qpos
        q_max = calibration.points[-1].qpos
        geometry_qpos = 0.5 * (q_min + q_max)
    else:
        geometry_qpos = config.preclamp_position
    geometry = interpolate_gripper_geometry(calibration, geometry_qpos)
    candidates, rejected, grid_count = _enumerate_tuna_candidates(
        node=node,
        config=config,
        bounds=bounds,
        table_z_m=table_z_m,
        calibration=calibration,
        geometry=geometry,
        require_moveit=not is_generation,
    )
    if not candidates:
        raise TunaGraspError(
            TunaErrorCode.PLANNING,
            "candidate_generation",
            {
                "object_name": "tuna_fish_can",
                "reason": "all_candidates_rejected",
                "grid_count": int(grid_count),
                "rejected_count": len(rejected),
            },
        )
    shared_debug = {
        "tuna_debug_stop_after": config.debug_stop_after,
        "tuna_candidate_grid_count": int(grid_count),
        "tuna_candidate_accepted_count": len(candidates),
        "tuna_candidate_rejections": list(rejected),
        "tuna_bounds_center": bounds.center.copy(),
        "tuna_bounds_size": bounds.size.copy(),
        "tuna_bounds_source_timestamps": list(bounds.source_timestamps),
        "tuna_bounds_received_monotonic": list(bounds.received_monotonic),
        "tuna_table_z_m": table_z_m,
        "tuna_calibration_sha256": calibration.source_sha256,
        "tuna_calibration_canonical_content_sha256": artifact[
            "canonical_content_sha256"
        ],
        "tuna_geometry_qpos": float(geometry_qpos),
        "tuna_perception_cache_invalidation_verified": (
            provenance.cache_invalidation_verified
        ),
        "tuna_perception_files": [dict(item) for item in provenance.files],
        "T_world_obj_raw": np.asarray(T_world_obj_raw, dtype=float).copy(),
    }
    results = []
    for ranked_index, candidate in enumerate(candidates, start=1):
        candidate_debug = dict(shared_debug)
        candidate_debug.update(
            {
                "tuna_candidate_rank": ranked_index,
                "tuna_radial_index": candidate.radial_index,
                "tuna_radial_direction_world": (
                    candidate.radial_direction.copy()
                ),
                "tuna_contact_height_m": candidate.contact_height_m,
                "tuna_pitch_deg": candidate.pitch_deg,
                "tuna_clearance_lower_bound_m": (
                    candidate.clearance_lower_bound_m
                ),
                "tuna_reachability_score": candidate.reachability_score,
            }
        )
        if is_generation:
            plan = _tuna_generation_plan(start_pose_6d, candidate, candidate_debug)
        else:
            plan = build_tuna_preclamp_plan(
                start_pose_6d=start_pose_6d,
                pregrasp_pose_6d=_transform_matrix_to_pose6d(
                    np.array(candidate.T_world_tcp_pregrasp, copy=True)
                ),
                contact_support_pose_6d=_transform_matrix_to_pose6d(
                    np.array(candidate.T_world_tcp_contact, copy=True)
                ),
                preclamp_position=config.preclamp_position,
                final_roll_angle_deg=config.roll_checkpoint_angles_deg[-1],
                debug_info=candidate_debug,
            )
        grasp_pose_6d = _transform_matrix_to_pose6d(
            np.array(candidate.T_world_tcp_contact, copy=True)
        )
        results.append(
            PickPlacePlanningResult(
                plan=plan,
                T_world_obj=np.asarray(T_world_obj_raw, dtype=float).copy(),
                T_world_grasp=np.array(candidate.T_world_tcp_contact, copy=True),
                grasp_pose_6d=grasp_pose_6d,
                candidate_index=ranked_index,
                candidate_count=len(candidates),
            )
        )
    return results


def plan_pick_place_candidates_from_perception(
    node,
    start_pose_6d,
    object_cam_pose_path=FOUNDATIONPOSE_OBJECT_CAMERA_POSE_JSON,
    selected_object_path=SELECTED_OBJECT_PATH,
    camera_frame=CAMERA_FRAME,
):
    selected_object_name = get_selected_object_info(selected_object_path)
    T_world_obj_raw = estimate_object_world_pose(
        node,
        object_cam_pose_path,
        camera_frame,
    )
    if _normalize_tuna_object_name(selected_object_name) == "tuna_fish_can":
        return _plan_tuna_fish_can_candidates(
            node=node,
            start_pose_6d=start_pose_6d,
            T_world_obj_raw=T_world_obj_raw,
            selected_object_path=selected_object_path,
            object_cam_pose_path=object_cam_pose_path,
        )
    T_world_obj = canonicalize_tabletop_object_pose(
        T_world_obj_raw,
        selected_object_name,
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
    print(f"Grasp execution mode: {GRASP_EXECUTION_MODE}")
    obstacles = _load_obstacles_for_place_and_clearance(
        node,
        selected_object_name,
        grasp_profile,
    )
    safe_place_selection = None
    if GRASP_EXECUTION_MODE == "safe_place" and GRASP_PLACE_ENABLED:
        if GRASP_PLACE_USE_SCENE:
            safe_place_selection = _select_safe_place_xy(obstacles)
        else:
            safe_place_selection = _select_safe_place_xy([])
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
        # ``select_grasp_pose_candidates_6d`` applies the configured profile
        # offset, and vertical bounds alignment may add a floor correction.
        # Recover the raw library Z for the grasp-pose debug log.
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
                "alignment_error_deg="
                f"{pear_close_metrics['alignment_error_deg']:.3f}, "
                f"projected_width_m={pear_close_metrics['projected_width_m']:.5f}, "
                f"opening_margin_m={pear_close_metrics['opening_margin_m']:.5f}, "
                "expected_position_rad="
                f"{pear_close_metrics['expected_position_rad']:.3f}, "
                "minimum_position_rad="
                f"{pear_close_metrics['minimum_position_rad']:.3f}"
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
            selected_object_name,
        )
        drop_pose_6d = (
            _build_safe_drop_pose_for_grasp(
                grasp_pose_6d,
                safe_place_selection,
            )
            if GRASP_EXECUTION_MODE == "safe_place"
            else None
        )
        plan = plan_safe_pick_place_steps(
            start_pose_6d=start_pose_6d,
            grasp_pose_6d=grasp_pose_6d,
            drop_pose_6d=drop_pose_6d,
            direct_to_grasp=grasp_profile == "side",
            execution_mode=GRASP_EXECUTION_MODE,
            debug_info={
                "object_name": selected_object_name,
                "grasp_profile": grasp_profile,
                "execution_mode": GRASP_EXECUTION_MODE,
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
                "safe_place_xy": (
                    None
                    if safe_place_selection is None
                    else safe_place_selection.xy.copy()
                ),
                "safe_place_min_clearance_m": (
                    None
                    if safe_place_selection is None
                    else safe_place_selection.min_clearance_m
                ),
                "grasp_lift_height_m": GRASP_LIFT_HEIGHT,
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
):
    return plan_pick_place_candidates_from_perception(
        node,
        start_pose_6d,
        object_cam_pose_path,
        selected_object_path,
        camera_frame,
    )[0]

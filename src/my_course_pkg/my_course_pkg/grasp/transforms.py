import json

import numpy as np
import rclpy
import tf2_ros
import tf_transformations
from sim_pick_place.utils.helpers import _transform_matrix_to_pose6d

from my_course_pkg.grasp.config import (
    CAMERA_POSE_CONVENTION,
    GRASP_CANONICALIZE_TABLETOP_OBJECT_POSE,
    TABLETOP_CANONICAL_OBJECTS,
)


WORLD_UP = np.array([0.0, 0.0, 1.0], dtype=float)


def load_object_cam_pose(json_path):
    with open(json_path, "r") as f:
        return np.array(json.load(f)["pose"], dtype=np.float32)


def pose_text(pose_6d):
    return (
        f"xyz: {np.array2string(np.asarray(pose_6d[:3]), precision=4)} "
        f"rpy: {np.array2string(np.asarray(pose_6d[3:6]), precision=4)}"
    )


def print_pose_summary(name, matrix):
    print(f"{name} {pose_text(_transform_matrix_to_pose6d(matrix))}")


def _normalize(vector):
    vector = np.asarray(vector, dtype=float)
    norm = np.linalg.norm(vector)
    if norm == 0.0:
        raise RuntimeError("Cannot normalize a zero-length vector.")
    return vector / norm


def _object_name_key(selected_object_name):
    if selected_object_name is None:
        return ""
    return str(selected_object_name).strip().lower().replace(" ", "_")


def object_local_z_axis_world(T_world_obj):
    """Return the object-local +Z axis expressed in world/base coordinates."""
    return _normalize(np.asarray(T_world_obj, dtype=float)[:3, :3] @ WORLD_UP)


def object_z_tilt_deg(T_world_obj):
    """Return angle between object-local +Z and world/base +Z."""
    z_axis = object_local_z_axis_world(T_world_obj)
    cos_angle = float(np.clip(z_axis @ WORLD_UP, -1.0, 1.0))
    return float(np.degrees(np.arccos(cos_angle)))


def should_canonicalize_tabletop_object_pose(selected_object_name):
    return (
        GRASP_CANONICALIZE_TABLETOP_OBJECT_POSE
        and _object_name_key(selected_object_name) in TABLETOP_CANONICAL_OBJECTS
    )


def _yaw_from_world_rotation(rotation):
    """Extract the world-Z yaw from an object's horizontal x-axis projection."""
    x_axis = np.asarray(rotation, dtype=float)[:3, 0].copy()
    x_axis[2] = 0.0
    if np.linalg.norm(x_axis) < 1e-6:
        y_axis = np.asarray(rotation, dtype=float)[:3, 1].copy()
        y_axis[2] = 0.0
        return float(np.arctan2(y_axis[1], y_axis[0]) - np.pi / 2.0)
    x_axis = _normalize(x_axis)
    return float(np.arctan2(x_axis[1], x_axis[0]))


def _world_z_yaw_rotation(yaw_rad):
    cos_yaw = float(np.cos(yaw_rad))
    sin_yaw = float(np.sin(yaw_rad))
    return np.array(
        [
            [cos_yaw, -sin_yaw, 0.0],
            [sin_yaw, cos_yaw, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def canonicalize_tabletop_object_pose(T_world_obj, selected_object_name):
    """Use an upright tabletop object frame for symmetric cylindrical cans.

    The translation stays exactly as detected.  For configured tabletop objects,
    roll/pitch from the perception result are replaced with world-up alignment
    while preserving the detected yaw around world/base +Z.
    """
    T_world_obj = np.asarray(T_world_obj, dtype=float)
    canonical = T_world_obj.copy()
    if not should_canonicalize_tabletop_object_pose(selected_object_name):
        return canonical

    yaw_rad = _yaw_from_world_rotation(T_world_obj[:3, :3])
    canonical[:3, :3] = _world_z_yaw_rotation(yaw_rad)
    canonical[:3, 3] = T_world_obj[:3, 3]
    return canonical


def transform_msg_to_matrix(transform):
    trans = transform.transform.translation
    rot = transform.transform.rotation
    matrix = np.eye(4)
    matrix[:3, 3] = [trans.x, trans.y, trans.z]
    matrix[:3, :3] = tf_transformations.quaternion_matrix(
        [rot.x, rot.y, rot.z, rot.w]
    )[:3, :3]
    return matrix


def camera_pose_convention_transform():
    """Return T_tf_camera_cv_camera for the FoundationPose output convention."""
    transform = np.eye(4)

    if CAMERA_POSE_CONVENTION in ("opencv", "optical", "identity"):
        return transform

    if CAMERA_POSE_CONVENTION in ("opencv_to_mujoco", "opencv_to_opengl"):
        transform[:3, :3] = np.diag([1.0, -1.0, -1.0])
        return transform

    raise ValueError(
        "Unsupported GRASP_CAMERA_POSE_CONVENTION="
        f"{CAMERA_POSE_CONVENTION!r}. Use 'opencv_to_mujoco' or 'identity'."
    )


def assert_valid_rotation(name, matrix):
    det = float(np.linalg.det(matrix[:3, :3]))
    if not np.isfinite(det) or abs(det - 1.0) > 1e-3:
        raise RuntimeError(
            f"{name} rotation is invalid: det={det:.6f}. "
            "This usually means a coordinate conversion used a reflection."
        )
    print(f"{name} rotation det: {det:.6f}")


def get_transform_checked(node, from_frame, to_frame, timeout_sec=2.0):
    timeout = rclpy.duration.Duration(seconds=timeout_sec)
    if not node.tf_buffer.can_transform(
        to_frame,
        from_frame,
        rclpy.time.Time(),
        timeout,
    ):
        raise RuntimeError(
            f"TF transform unavailable: {from_frame} -> {to_frame}. "
            "Do not use identity here; check the camera frame name and TF tree."
        )

    try:
        transform = node.tf_buffer.lookup_transform(
            to_frame,
            from_frame,
            rclpy.time.Time(),
        )
    except tf2_ros.TransformException as exc:
        raise RuntimeError(
            f"TF lookup failed: {from_frame} -> {to_frame}: {exc}"
        ) from exc

    return transform_msg_to_matrix(transform)


def estimate_object_world_pose(node, object_cam_pose_path, camera_frame):
    T_cv_obj = load_object_cam_pose(object_cam_pose_path)
    assert_valid_rotation("foundationpose T_cv_obj", T_cv_obj)

    T_tf_cam_cv_cam = camera_pose_convention_transform()
    assert_valid_rotation("camera convention transform", T_tf_cam_cv_cam)
    T_cam_obj = T_tf_cam_cv_cam @ T_cv_obj
    assert_valid_rotation("converted T_cam_obj", T_cam_obj)

    print(f"Camera pose convention: {CAMERA_POSE_CONVENTION}")
    print_pose_summary("object camera raw(cv)", T_cv_obj)
    print_pose_summary("object camera converted(tf)", T_cam_obj)

    T_world_cam = get_transform_checked(node, camera_frame, "world")
    assert_valid_rotation("T_world_cam", T_world_cam)
    T_world_obj = T_world_cam @ T_cam_obj
    assert_valid_rotation("T_world_obj", T_world_obj)
    return T_world_obj

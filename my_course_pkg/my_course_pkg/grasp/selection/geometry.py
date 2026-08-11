"""Small transform and geometry helpers shared by grasp profiles."""

import numpy as np


WORLD_DOWN = np.array([0.0, 0.0, -1.0])
CENTER_OFFSET_SCORE_WEIGHT = 250.0


def _normalize(vector):
    vector = np.asarray(vector, dtype=float)
    norm = np.linalg.norm(vector)
    if norm == 0.0:
        raise RuntimeError("Cannot normalize a zero-length vector.")
    return vector / norm


def _angle_deg(vector, target):
    vector = _normalize(vector)
    target = _normalize(target)
    cos_angle = float(np.clip(vector @ target, -1.0, 1.0))
    return float(np.degrees(np.arccos(cos_angle)))


def _tool_z_axis_world(transform_matrix):
    return _normalize(transform_matrix[:3, :3] @ np.array([0.0, 0.0, 1.0]))


def tool_z_down_angle_deg(transform_matrix):
    return _angle_deg(_tool_z_axis_world(transform_matrix), WORLD_DOWN)


def _object_side_axes_world(T_world_obj):
    rot = T_world_obj[:3, :3]
    return [
        rot @ np.array([1.0, 0.0, 0.0]),
        rot @ np.array([-1.0, 0.0, 0.0]),
        rot @ np.array([0.0, 1.0, 0.0]),
        rot @ np.array([0.0, -1.0, 0.0]),
    ]


def _min_axis_angle_deg(axis, targets):
    return min(_angle_deg(axis, target) for target in targets)


def _closest_side_axis_index(axis, T_world_obj):
    side_axes = _object_side_axes_world(T_world_obj)
    return int(np.argmin([_angle_deg(axis, target) for target in side_axes]))


def _object_z_rotation(angle_deg):
    angle_rad = np.deg2rad(float(angle_deg))
    cos_angle = np.cos(angle_rad)
    sin_angle = np.sin(angle_rad)

    transform = np.eye(4)
    transform[:3, :3] = np.array(
        [
            [cos_angle, -sin_angle, 0.0],
            [sin_angle, cos_angle, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    return transform


def _object_translation(offset):
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(offset, dtype=float)
    return transform


def _object_z_rotation_about_point(angle_deg, point):
    point = np.asarray(point, dtype=float)
    return (
        _object_translation(point)
        @ _object_z_rotation(angle_deg)
        @ _object_translation(-point)
    )


def _object_point_in_tcp(T_obj_grasp, point_obj):
    T_obj_grasp = np.asarray(T_obj_grasp, dtype=float)
    point_obj = np.asarray(point_obj, dtype=float)
    rotation = T_obj_grasp[:3, :3]
    translation = T_obj_grasp[:3, 3]
    return rotation.T @ (point_obj - translation)

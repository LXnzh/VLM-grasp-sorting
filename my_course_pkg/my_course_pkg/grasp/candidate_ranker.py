from dataclasses import dataclass
from pathlib import Path

import numpy as np


WORLD_DOWN = np.array([0.0, 0.0, -1.0], dtype=float)


@dataclass(frozen=True)
class RankedGrasp:
    index: int
    T_object_grasp: np.ndarray
    T_world_grasp: np.ndarray
    approach_angle_deg: float


def _validate_pose_batch(poses, source):
    poses = np.asarray(poses, dtype=float)
    if poses.ndim != 3 or poses.shape[1:] != (4, 4):
        raise ValueError(
            f"{source} poses must have shape (N, 4, 4), got {poses.shape}."
        )
    if poses.shape[0] == 0:
        raise ValueError(f"{source} contains no poses.")
    if not np.isfinite(poses).all():
        raise ValueError(f"{source} poses contain non-finite values.")
    if not np.allclose(poses[:, 3, :], [0.0, 0.0, 0.0, 1.0]):
        raise ValueError(f"{source} poses must be homogeneous transforms.")
    return poses


def load_grasp_candidates(grasp_dir):
    grasp_dir = Path(grasp_dir)
    archives = sorted(grasp_dir.glob("*.npz"))
    if not archives:
        raise FileNotFoundError(f"No .npz grasp archives found in {grasp_dir}.")

    batches = []
    for archive_path in archives:
        with np.load(archive_path) as archive:
            if "poses" not in archive.files:
                raise ValueError(f"{archive_path} does not contain a 'poses' array.")
            batches.append(_validate_pose_batch(archive["poses"], archive_path))
    return np.concatenate(batches, axis=0)


def rank_grasp_candidates(T_world_object, T_object_grasps, top_k=None):
    T_world_object = _validate_pose_batch(
        np.asarray(T_world_object, dtype=float)[None, :, :], "T_world_object"
    )[0]
    T_object_grasps = _validate_pose_batch(T_object_grasps, "T_object_grasps")
    if top_k is not None and top_k <= 0:
        raise ValueError("top_k must be positive when provided.")

    T_world_grasps = T_world_object @ T_object_grasps
    tool_z_axes = T_world_grasps[:, :3, 2]
    norms = np.linalg.norm(tool_z_axes, axis=1)
    if np.any(norms == 0.0):
        raise ValueError("A grasp pose has a zero-length tool Z axis.")
    cosines = np.clip((tool_z_axes / norms[:, None]) @ WORLD_DOWN, -1.0, 1.0)
    angles = np.degrees(np.arccos(cosines))
    ordered_indices = np.argsort(angles, kind="stable")
    if top_k is not None:
        ordered_indices = ordered_indices[:top_k]

    return [
        RankedGrasp(
            index=int(index),
            T_object_grasp=T_object_grasps[index].copy(),
            T_world_grasp=T_world_grasps[index].copy(),
            approach_angle_deg=float(angles[index]),
        )
        for index in ordered_indices
    ]

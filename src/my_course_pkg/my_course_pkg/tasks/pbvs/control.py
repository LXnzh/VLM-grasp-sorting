"""ROS-independent safety checks and control math for camera PBVS."""

from collections import deque

import numpy as np

from my_course_pkg.tasks.tracking.motion_gate import MOVING, STABILIZING


class PBVSError(RuntimeError):
    """Raised when a safe PBVS command cannot be produced."""


class PositionWindowStabilityGate:
    """Require a bounded 3D position span over a fresh-sample window."""

    def __init__(self, max_span_m, duration_s, minimum_samples):
        self.max_span_m = float(max_span_m)
        self.duration_s = float(duration_s)
        self.minimum_samples = int(minimum_samples)
        if not np.isfinite(self.max_span_m) or self.max_span_m < 0.0:
            raise ValueError(
                "Position stability span must be finite and non-negative."
            )
        if not np.isfinite(self.duration_s) or self.duration_s < 0.0:
            raise ValueError(
                "Position stability duration must be finite and non-negative."
            )
        if self.minimum_samples < 2:
            raise ValueError(
                "Position stability requires at least two samples."
            )
        self.reset()

    def reset(self):
        self.samples = deque()
        self.span_m = float("inf")
        self.covered_duration_s = 0.0

    def update(self, stamp, position):
        """Return whether the current time window is stable and its span."""
        stamp = float(stamp)
        position = np.asarray(position, dtype=float).reshape(3)
        if not np.isfinite(stamp) or not np.all(np.isfinite(position)):
            raise PBVSError(
                "Position stability samples must contain finite values."
            )
        if self.samples and stamp <= self.samples[-1][0]:
            return False, self.span_m

        self.samples.append((stamp, position.copy()))
        cutoff = stamp - self.duration_s
        while len(self.samples) > 1 and self.samples[1][0] <= cutoff:
            self.samples.popleft()

        positions = np.stack(
            [sample_position for _, sample_position in self.samples], axis=0
        )
        differences = positions[:, None, :] - positions[None, :, :]
        self.span_m = float(np.max(np.linalg.norm(differences, axis=2)))
        self.covered_duration_s = stamp - self.samples[0][0]
        stable = (
            self.covered_duration_s >= self.duration_s
            and len(self.samples) >= self.minimum_samples
            and self.span_m <= self.max_span_m
        )
        return stable, self.span_m


def masked_depth_centroid(depth, mask, camera_matrix, minimum_pixels=30):
    """Return the robust OpenCV-camera XYZ centroid of a tracked mask."""
    depth = np.asarray(depth, dtype=float)
    mask = np.asarray(mask, dtype=bool)
    matrix = np.asarray(camera_matrix, dtype=float).reshape(3, 3)
    if depth.shape != mask.shape:
        raise PBVSError(
            "Depth/mask shape mismatch: "
            f"depth={depth.shape}, mask={mask.shape}."
        )
    valid = mask & np.isfinite(depth) & (depth > 0.05) & (depth < 5.0)
    ys, xs = np.where(valid)
    if len(xs) < int(minimum_pixels):
        raise PBVSError(
            f"Tracked target has only {len(xs)} valid depth pixels; "
            f"required={int(minimum_pixels)}."
        )
    z = float(np.median(depth[valid]))
    u = float(np.median(xs))
    v = float(np.median(ys))
    fx, fy = float(matrix[0, 0]), float(matrix[1, 1])
    cx, cy = float(matrix[0, 2]), float(matrix[1, 2])
    if fx <= 0.0 or fy <= 0.0:
        raise PBVSError("Camera focal lengths must be positive.")
    return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z], dtype=float)


def target_observation_jump_m(previous, current):
    """Return the finite 3D displacement between target observations."""
    previous = np.asarray(previous, dtype=float).reshape(3)
    current = np.asarray(current, dtype=float).reshape(3)
    if not np.all(np.isfinite(previous)) or not np.all(np.isfinite(current)):
        raise PBVSError("Target observations must contain only finite values.")
    return float(np.linalg.norm(current - previous))


def _bounded_proportional_step(error, gain, deadband_m, max_step_m):
    error = np.asarray(error, dtype=float).reshape(3)
    gain = float(gain)
    deadband_m = float(deadband_m)
    max_step_m = float(max_step_m)
    if not np.all(np.isfinite(error)):
        raise PBVSError("PBVS error must contain only finite values.")
    if not np.isfinite(gain):
        raise PBVSError("PBVS gain must be finite.")
    if not np.isfinite(deadband_m):
        raise PBVSError("PBVS deadband must be finite.")
    if not np.isfinite(max_step_m):
        raise PBVSError("PBVS maximum step must be finite.")
    if gain < 0.0:
        raise PBVSError("PBVS gain must be non-negative.")
    if deadband_m < 0.0:
        raise PBVSError("PBVS deadband must be non-negative.")
    if max_step_m <= 0.0:
        raise PBVSError("PBVS maximum step must be positive.")
    if float(np.linalg.norm(error)) <= deadband_m:
        return np.zeros(3, dtype=float), False
    if gain == 0.0:
        return np.zeros(3, dtype=float), False
    step = gain * error
    step_norm = float(np.linalg.norm(step))
    if step_norm > max_step_m:
        step *= max_step_m / step_norm
    return step, True


def should_run_pbvs(state):
    """Keep following only while the camera-relative target is in motion."""
    return state in {MOVING, STABILIZING}


def continuous_stop_ready(target_stable, tcp_stable):
    """Allow grasping from stability alone, regardless of initial offset."""
    return bool(target_stable and tcp_stable)


def _transform_point(transform, point):
    transform = np.asarray(transform, dtype=float).reshape(4, 4)
    if not np.all(np.isfinite(transform)):
        raise PBVSError("PBVS transform must contain only finite values.")
    point_h = np.ones(4, dtype=float)
    point_h[:3] = np.asarray(point, dtype=float).reshape(3)
    if not np.all(np.isfinite(point_h)):
        raise PBVSError("PBVS point must contain only finite values.")
    transformed = (transform @ point_h)[:3]
    if not np.all(np.isfinite(transformed)):
        raise PBVSError("PBVS transformed point must contain only finite values.")
    return transformed


def _check_workspace(
    command_xyz,
    *,
    T_workspace_command,
    workspace_min,
    workspace_max,
    command_frame,
    workspace_frame,
):
    command_xyz = np.asarray(command_xyz, dtype=float).reshape(3)
    command_workspace = _transform_point(T_workspace_command, command_xyz)
    lower = np.asarray(workspace_min, dtype=float).reshape(3)
    upper = np.asarray(workspace_max, dtype=float).reshape(3)
    if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)):
        raise PBVSError("PBVS workspace bounds must be finite.")
    if np.any(lower >= upper):
        raise PBVSError(
            "PBVS workspace bounds are invalid: "
            f"min={lower.tolist()}, max={upper.tolist()}."
        )
    if np.any(command_workspace < lower) or np.any(command_workspace > upper):
        raise PBVSError(
            "PBVS command is outside the configured workspace: "
            f"command_{command_frame}={command_xyz.tolist()}, "
            f"command_{workspace_frame}={command_workspace.tolist()}, "
            f"min_{workspace_frame}={lower.tolist()}, "
            f"max_{workspace_frame}={upper.tolist()}."
        )
    return command_workspace


def bounded_camera_pbvs_target(
    current_world_xyz,
    target_camera_xyz,
    reference_camera_xyz,
    *,
    T_world_cv_camera,
    T_workspace_world,
    gain,
    deadband_m,
    max_step_m,
    workspace_min,
    workspace_max,
    workspace_frame,
):
    """Convert a bounded camera-relative PBVS step into a world command."""
    current_world_xyz = np.asarray(current_world_xyz, dtype=float).reshape(3)
    target_camera_xyz = np.asarray(target_camera_xyz, dtype=float).reshape(3)
    reference_camera_xyz = np.asarray(reference_camera_xyz, dtype=float).reshape(3)
    error_camera = target_camera_xyz - reference_camera_xyz
    step_camera, should_command = _bounded_proportional_step(
        error_camera, gain, deadband_m, max_step_m
    )
    if not should_command:
        return current_world_xyz.copy(), error_camera, False

    T_world_cv_camera = np.asarray(T_world_cv_camera, dtype=float).reshape(4, 4)
    if not np.all(np.isfinite(T_world_cv_camera)):
        raise PBVSError(
            "PBVS camera-to-world transform must contain finite values."
        )
    R_world_camera = T_world_cv_camera[:3, :3]
    step_world = R_world_camera @ step_camera
    command_world = current_world_xyz + step_world
    _check_workspace(
        command_world,
        T_workspace_command=T_workspace_world,
        workspace_min=workspace_min,
        workspace_max=workspace_max,
        command_frame="world",
        workspace_frame=workspace_frame,
    )
    return command_world, error_camera, True

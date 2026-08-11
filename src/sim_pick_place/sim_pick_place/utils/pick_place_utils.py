from sim_pick_place.utils.helpers import Pose6D, _ensure_pose_array, _rpy_to_rot, _rot_to_rpy, _z_axis_from_rot
import numpy as np
from scipy.spatial.transform import Slerp, Rotation as R
from typing import List




def get_pregrasp_pose(grasp_pose_6d: Pose6D, dist: float) -> Pose6D:
    """
    Return a new pose that is `dist` meters along the LOCAL +Z axis
    of the given grasp pose (i.e., move along the tool's z-axis).
    Orientation is unchanged.
    """
    grasp_pose_6d = _ensure_pose_array(grasp_pose_6d)

    # Local +Z in world coords from current orientation
    rot = _rpy_to_rot(*grasp_pose_6d[3:])       # Rz(yaw)*Ry(pitch)*Rx(roll)
    z_axis_world = _z_axis_from_rot(rot)        # shape (3,)

    pregrasp_pose_6d = grasp_pose_6d.copy()
    pregrasp_pose_6d[:3] += float(dist) * z_axis_world
    return pregrasp_pose_6d

def interpolate_lin(start_pose_6d: Pose6D, end_pose_6d: Pose6D, max_dist: float) -> List[Pose6D]:
    """
    Linear interpolation between two poses.
    - Position: linear in XYZ, with spacing so that successive points are <= max_dist apart in XYZ.
    - Orientation: SLERP between start and end orientations (full 3D), not per-angle.
    Returns a list including the start and end poses as (6,) numpy arrays.
    """
    start_pose_6d = _ensure_pose_array(start_pose_6d)
    end_pose_6d   = _ensure_pose_array(end_pose_6d)

    s_xyz, e_xyz = start_pose_6d[:3], end_pose_6d[:3]
    d_xyz = e_xyz - s_xyz
    dist_xyz = float(np.linalg.norm(d_xyz))
    steps = max(1, int(np.ceil(dist_xyz / max_dist))) if max_dist > 0 else 1

    # SLERP setup
    r0 = _rpy_to_rot(*start_pose_6d[3:])
    r1 = _rpy_to_rot(*end_pose_6d[3:])
    key_times = np.array([0.0, 1.0], dtype=float)
    key_rots  = R.from_quat([r0.as_quat(), r1.as_quat()])
    slerp = Slerp(key_times, key_rots)

    ts = np.linspace(0.0, 1.0, steps + 1)
    out: List[Pose6D] = []
    for t in ts:
        p = s_xyz + d_xyz * t
        rot_t = slerp([t])[0]
        rpy_t = _rot_to_rpy(rot_t)
        pose_t = np.hstack([p, rpy_t])
        out.append(pose_t.astype(float))
    return out


def interpolate_arc(start_pose_6d: Pose6D, end_pose_6d: Pose6D, max_dist: float) -> List[Pose6D]:
    """
    Interpolate along an upward 'table arc' between two poses, assuming the tool's
    local +Z points downward (opposite to global +Z).

    Preconditions:
      - The local +Z axis of both start and end orientations must be within ±10° of GLOBAL -Z
        (i.e., gripper looking down). Otherwise, raises ValueError.

    Path:
      - XY: straight line from start to end.
      - Z: linear interpolation + a half-sine bump peaking at the midpoint:
            z(t) = z_lin(t) + lift * sin(pi * t),
        where lift = max(0.05 m, 0.5 * horizontal_distance). (Arc lifts upward.)

    Orientation:
      - Full 3D SLERP between start and end orientations.

    Spacing:
      - Segment lengths along the spatial path are chosen so consecutive XYZ points are <= max_dist.

    Returns list including start and end poses as (6,) numpy arrays.
    """
    start_pose_6d = _ensure_pose_array(start_pose_6d)
    end_pose_6d   = _ensure_pose_array(end_pose_6d)

    # Alignment checks (±10° to GLOBAL −Z)
    r_start = _rpy_to_rot(*start_pose_6d[3:])
    r_end   = _rpy_to_rot(*end_pose_6d[3:])
    minus_Z = np.array([0.0, 0.0, -1.0], dtype=float)
    for tag, rot in (("start", r_start), ("end", r_end)):
        z_local = _z_axis_from_rot(rot)               # world coords of local +Z (tool axis)
        cosang = float(np.clip(z_local @ minus_Z, -1.0, 1.0))
        ang = float(np.arccos(cosang))                # angle to GLOBAL −Z
        if ang > np.deg2rad(10.0):
            raise ValueError(
                f"{tag} pose z-axis not aligned with global -Z "
                f"(angle={np.degrees(ang):.2f} deg > 10 deg)."
            )

    s_xyz, e_xyz = start_pose_6d[:3], end_pose_6d[:3]
    # Horizontal distance (for lift heuristic)
    hdist = float(np.linalg.norm((e_xyz - s_xyz)[:2]))
    lift = max(0.05, 0.5 * hdist)

    # Parametric spatial path (upward arc)
    def pos_at(t: float) -> np.ndarray:
        xy = s_xyz[:2] + (e_xyz[:2] - s_xyz[:2]) * t
        z_lin = s_xyz[2] + (e_xyz[2] - s_xyz[2]) * t
        z = z_lin + lift * np.sin(np.pi * t)          # arc points upward (+Z)
        return np.array([xy[0], xy[1], z], dtype=float)

    # Estimate arc length (sample) to set step count
    samples = 20
    pts = np.array([pos_at(i / samples) for i in range(samples + 1)])
    segs = np.linalg.norm(pts[1:] - pts[:-1], axis=1)
    length = float(np.sum(segs))
    steps = max(1, int(np.ceil(length / max_dist))) if max_dist > 0 else 1

    # SLERP across the same t grid
    key_times = np.array([0.0, 1.0], dtype=float)
    key_rots  = R.from_quat([r_start.as_quat(), r_end.as_quat()])
    slerp = Slerp(key_times, key_rots)

    ts = np.linspace(0.0, 1.0, steps + 1)
    out: List[Pose6D] = []
    for t in ts:
        p = pos_at(float(t))
        rot_t = slerp([t])[0]
        rpy_t = _rot_to_rpy(rot_t)
        pose_t = np.hstack([p, rpy_t])
        out.append(pose_t.astype(float))
    return out


def generate_random_pose_here(initial_pose_6d: Pose6D, range_pose_6d: Pose6D) -> Pose6D:
    """
    Generate a random pose near `initial_pose_6d`, bounded by per-component ranges in `range_pose_6d`.

    For each component i:
      - If range_pose_6d[i] == 0, the returned component equals initial_pose_6d[i].
      - Else, sample uniformly from [initial - range, initial + range].

    Angles are wrapped by reconstructing a Rotation and re-extracting RPY, which
    normalizes them to a consistent principal-value representation.
    """
    initial_pose_6d = _ensure_pose_array(initial_pose_6d)
    range_pose_6d   = _ensure_pose_array(range_pose_6d)

    out = np.empty(6, dtype=float)

    # XYZ
    for i in range(3):
        if range_pose_6d[i] == 0.0:
            out[i] = initial_pose_6d[i]
        else:
            out[i] = float(np.random.uniform(initial_pose_6d[i] - range_pose_6d[i],
                                             initial_pose_6d[i] + range_pose_6d[i]))

    # RPY — sample, then normalize via Rotation
    sampled_rpy = np.empty(3, dtype=float)
    for i in range(3):
        idx = 3 + i
        if range_pose_6d[idx] == 0.0:
            sampled_rpy[i] = initial_pose_6d[idx]
        else:
            sampled_rpy[i] = float(np.random.uniform(initial_pose_6d[idx] - range_pose_6d[idx],
                                                     initial_pose_6d[idx] + range_pose_6d[idx]))
    # Normalize angles
    r = _rpy_to_rot(*sampled_rpy)
    out[3:] = _rot_to_rpy(r)

    random_pose_6d: Pose6D = out
    return random_pose_6d


def plan_pick_place_trajectory(
    start_pose_6d: Pose6D,
    object_pose_6d: Pose6D,
    drop_pose_6d: Pose6D,
    approach_dist: float,
    max_dist: float,
    include_retreat: bool = True,
    add_gripper_state: bool = True,
) -> List[Pose6D]:
    """
    Create a full pick→place trajectory.

    Inputs:
      - start_pose_6d : robot current pose (np.ndarray, shape (6,))
      - object_pose_6d: grasp pose at the object (tool +Z down)
      - drop_pose_6d  : final drop pose (slightly higher, some distance away; tool +Z down)
      - approach_dist : positive distance to stay above (opposite tool +Z) for approach/retract [m]
      - max_dist      : max linear spacing between consecutive waypoints [m]
      - include_retreat: if True, retreat back to pre-drop after the drop

    Returns:
      - List[np.ndarray] of poses (each shape (6,)), concatenated in order:
          start → pre-object → object → (upward arc) → pre-drop → drop → [pre-drop if retreat]
    """
    # Ensure arrays (uses your helper if available)
    start_pose_6d  = np.asarray(start_pose_6d, dtype=float).reshape(6)
    object_pose_6d = np.asarray(object_pose_6d, dtype=float).reshape(6)
    drop_pose_6d   = np.asarray(drop_pose_6d, dtype=float).reshape(6)

    # Pre-poses: move opposite tool +Z (i.e., '-approach_dist' along local +Z)
    pre_object_pose_6d = get_pregrasp_pose(object_pose_6d, -float(approach_dist))
    pre_drop_pose_6d   = get_pregrasp_pose(drop_pose_6d,   -float(approach_dist))

    # Plan segments
    seg1 = interpolate_lin(start_pose_6d,       pre_object_pose_6d, max_dist)  # approach to above object
    seg2 = interpolate_lin(pre_object_pose_6d,  object_pose_6d,     max_dist)  # descend to grasp
    seg3 = interpolate_lin(object_pose_6d,      pre_object_pose_6d, max_dist)  # lift to pre-object
    seg4 = interpolate_arc(pre_object_pose_6d,      pre_drop_pose_6d,   max_dist)  # move toward drop
    seg5 = interpolate_lin(pre_drop_pose_6d,    drop_pose_6d,       max_dist)  # descend to drop

    # Optional retreat back to pre-drop after releasing
    seg6 = interpolate_lin(drop_pose_6d,        pre_drop_pose_6d,   max_dist) if include_retreat else []

    # Concatenate while avoiding duplicate endpoints between segments
    def _concat_no_dup(parts: List[List[Pose6D]], gripper_state: List[bool]) -> List[Pose6D]:
        gripper_open_state = 0.0
        gripper_closed_state = 0.59
        out: List[Pose6D] = []
        for part, state in zip(parts, gripper_state):
            if not part:
                continue
            part_modified: List[Pose6D] = []
            if add_gripper_state:
                for pose in part:
                    gripper_state = gripper_open_state if state else gripper_closed_state
                    pose_with_gripper = np.hstack([pose, [gripper_state]])  # append gripper state as last element
                    part_modified.append(pose_with_gripper)
            else:
                part_modified = part
            if not out:
                out.extend(part_modified)
            else:
                # skip first element if it equals the last of out (within numeric tolerance)
                first = part_modified[0]
                if np.allclose(out[-1], first, atol=1e-9):
                    out.extend(part_modified[1:])
                else:
                    out.extend(part_modified)
        return out
    gripper_state_open = [True, True, False, False, False, True]
    trajectory = _concat_no_dup([seg1, seg2, seg3, seg4, seg5, seg6], gripper_state_open)
    return trajectory


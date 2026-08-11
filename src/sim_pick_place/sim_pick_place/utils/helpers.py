import numpy as np
from scipy.spatial.transform import Rotation as R


Pose6D = np.ndarray  # shape (6,), [x, y, z, roll, pitch, yaw]


def _ensure_pose_array(pose_6d) -> Pose6D:
    """Ensure a (6,) float64 numpy array."""
    a = np.asarray(pose_6d, dtype=float).reshape(6)
    return a


def _rpy_to_rot(roll: float, pitch: float, yaw: float) -> R:
    """
    Convert RPY (roll, pitch, yaw) to Rotation using the convention:
    R = Rz(yaw) * Ry(pitch) * Rx(roll).
    """
    return R.from_euler("zyx", [yaw, pitch, roll], degrees=False)


def _rot_to_rpy(rot: R) -> np.ndarray:
    """Inverse of _rpy_to_rot; returns [roll, pitch, yaw]."""
    yaw, pitch, roll = rot.as_euler("zyx", degrees=False)
    return np.array([roll, pitch, yaw], dtype=float)


def _z_axis_from_rot(rot: R) -> np.ndarray:
    """World coords of local +Z for the rotation."""
    return rot.as_matrix()[:, 2]  # third column


def _angle_to_global_z(rot: R) -> float:
    """Angle (radians) between local +Z and global +Z."""
    z_local = _z_axis_from_rot(rot)
    cosang = np.clip(z_local @ np.array([0.0, 0.0, 1.0]), -1.0, 1.0)
    return float(np.arccos(cosang))

def _pose6d_to_posestamped_msg(pose_6d: Pose6D, frame_id: str):
    """Convert a (6,) pose to a geometry_msgs/PoseStamped message."""
    from geometry_msgs.msg import PoseStamped, Point, Quaternion
    pose_6d = _ensure_pose_array(pose_6d)
    position = Point(x=pose_6d[0], y=pose_6d[1], z=pose_6d[2])
    rot = _rpy_to_rot(pose_6d[3], pose_6d[4], pose_6d[5])
    quat = rot.as_quat()  # [x, y, z, w]
    orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = frame_id
    pose_msg.pose.position = position
    pose_msg.pose.orientation = orientation
    return pose_msg

def _posestamped_msg_to_pose6d(pose_msg) -> Pose6D:
    """Convert a geometry_msgs/PoseStamped message to a (6,) pose."""
    position = pose_msg.pose.position
    orientation = pose_msg.pose.orientation
    rot = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
    roll, pitch, yaw = rot.as_euler("zyx", degrees=False)[::-1]
    pose_6d = np.array([position.x, position.y, position.z, roll, pitch, yaw], dtype=float)
    return pose_6d


def _to_pose6d(position, orientation) -> Pose6D:
    """Convert position (3,) and orientation (quaternion (4,) or R) to (6,) pose.
    
    quaternion is in [w, x, y, z] format.
    
    """
    try:
        position = np.asarray(position, dtype=float).reshape(3)
        if isinstance(orientation, R):
            rot = orientation
        else:
            quat = np.asarray(orientation, dtype=float).reshape(4)
            # scipy's convention is [x, y, z, w]
            #quat = np.array([quat[1], quat[2], quat[3], quat[0]], dtype=float)
            rot = R.from_quat(quat)
        roll, pitch, yaw = rot.as_euler("zyx", degrees=False)[::-1]
        pose_6d = np.array([position[0], position[1], position[2], roll, pitch, yaw], dtype=float)
    except Exception as e:
        print(f"Error converting to pose6d: {e}")
        pose_6d = np.zeros(6, dtype=float)
    return pose_6d

def _pose6d_to_transform_matrix(pose_6d: Pose6D) -> np.ndarray:
    """Convert (6,) pose to a 4x4 transformation matrix."""
    pose_6d = _ensure_pose_array(pose_6d)
    T = np.eye(4, dtype=float)
    T[0:3, 3] = pose_6d[0:3]
    rot = _rpy_to_rot(pose_6d[3], pose_6d[4], pose_6d[5])
    T[0:3, 0:3] = rot.as_matrix()
    return T

def _transform_matrix_to_pose6d(T: np.ndarray) -> Pose6D:
    """Convert a 4x4 transformation matrix to (6,) pose."""
    T = np.asarray(T, dtype=float).reshape(4, 4)
    position = T[0:3, 3]
    rot = R.from_matrix(T[0:3, 0:3])
    roll, pitch, yaw = rot.as_euler("zyx", degrees=False)[::-1]
    pose_6d = np.array([position[0], position[1], position[2], roll, pitch, yaw], dtype=float)
    return pose_6d

def _randomize_orientation_z(q, angle_range=(0, 2*np.pi)):
    """
    Randomizes the orientation of quaternion `q` around the Z-axis
    within a given angular range (in radians).

    Parameters
    ----------
    q : list or np.ndarray
        The input quaternion [w, x, y, z].
    angle_range : tuple(float, float), optional
        The (min_angle, max_angle) range in radians for random rotation around Z.
        Defaults to (0, 2π).

    Returns
    -------
    list
        A new quaternion representing the input orientation rotated by
        a random angle around Z within the specified range.
    """
    q = np.array(q, dtype=float)

    # Generate random angle within given range
    theta = np.random.uniform(angle_range[0], angle_range[1])

    # Quaternion for rotation around Z-axis
    qz = np.array([np.cos(theta / 2), 0, 0, np.sin(theta / 2)])

    # Quaternion multiplication: q_new = qz * q
    w1, x1, y1, z1 = qz
    w2, x2, y2, z2 = q

    q_new = np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

    # Normalize
    q_new /= np.linalg.norm(q_new)

    return [q_new[0], q_new[1], q_new[2], q_new[3]]
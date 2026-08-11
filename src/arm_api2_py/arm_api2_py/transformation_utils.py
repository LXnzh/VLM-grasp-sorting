import numpy as np
import tf_transformations
from geometry_msgs.msg import Pose, PoseStamped, Transform, TransformStamped
from std_msgs.msg import Header


def extract_pos_quat_from_pose(pose: Pose) -> tuple[np.ndarray, np.ndarray]:
    """Extracts position and quaternion from a Pose.
    
    Returns:
        A tuple containing:
            - position: A numpy array of shape (3,) representing the position (x, y, z).
            - quaternion: A numpy array of shape (4,) representing the orientation (x, y, z, w).
    
    """
    position = np.array([pose.position.x, pose.position.y, pose.position.z])
    quaternion = np.array([pose.orientation.x, pose.orientation.y,
                           pose.orientation.z, pose.orientation.w])
    return position, quaternion

def pos_quat_to_pose(position: np.ndarray, quaternion: np.ndarray) -> Pose:
    """Creates a Pose from position and quaternion.
    
    Args:
        position: A numpy array of shape (3,) representing the position (x, y, z).
        quaternion: A numpy array of shape (4,) representing the orientation (x, y, z, w).
    Returns:
        A Pose object constructed from the given position and quaternion.
    
    """
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose

def pose_to_posestamped(pose: Pose, frame_id: str) -> PoseStamped:
    """Converts a Pose to a PoseStamped with the given frame_id."""
    return PoseStamped(
        header=Header(frame_id=frame_id),
        pose=pose
    )


def posestamped_to_pose(pose_stamped: PoseStamped) -> Pose:
    """Extracts a Pose from a PoseStamped."""
    return pose_stamped.pose


def pose_to_transform(pose: Pose) -> Transform:
    """Converts a Pose to a Transform."""
    transform = Transform()
    transform.translation.x = pose.position.x
    transform.translation.y = pose.position.y
    transform.translation.z = pose.position.z
    transform.rotation = pose.orientation
    return transform


def transform_to_transformstamped(transform: Transform,
                                  frame_id: str,
                                  child_frame_id: str) -> TransformStamped:
    """Converts a Transform to a TransformStamped with given frame_id and child_frame_id."""
    return TransformStamped(
        header=Header(frame_id=frame_id),
        child_frame_id=child_frame_id,
        transform=transform
    )


def transformstamped_to_transform(transform_stamped: TransformStamped) -> Transform:
    """Extracts a Transform from a TransformStamped."""
    return transform_stamped.transform


def numpy_to_transform(matrix: np.ndarray) -> Transform:
    """Converts a 4x4 numpy transformation matrix to a Transform."""
    translation = matrix[:3, 3]
    rotation = tf_transformations.quaternion_from_matrix(matrix)

    tf = Transform()
    tf.translation.x, tf.translation.y, tf.translation.z = translation
    tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w = rotation

    return tf


def transform_to_numpy(transform: Transform) -> np.ndarray:
    """Converts a Transform to a 4x4 numpy transformation matrix."""
    translation = [transform.translation.x,
                   transform.translation.y, transform.translation.z]
    rotation = [transform.rotation.x, transform.rotation.y,
                transform.rotation.z, transform.rotation.w]

    matrix = tf_transformations.quaternion_matrix(rotation)
    matrix[:3, 3] = translation

    return matrix


def numpy_from_translation_euler(translation: np.ndarray, euler_angles: np.ndarray) -> np.ndarray:
    """Generates a 4x4 numpy transformation matrix from translation and Euler angles."""
    matrix = tf_transformations.euler_matrix(*euler_angles)
    matrix[:3, 3] = translation
    return matrix

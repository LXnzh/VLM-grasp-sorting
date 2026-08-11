import numpy as np
import pytest
from geometry_msgs.msg import Pose, PoseStamped, Transform, TransformStamped
from arm_api2_py.transformation_utils import (
    pose_to_posestamped,
    posestamped_to_pose,
    pose_to_transform,
    transform_to_transformstamped,
    transformstamped_to_transform,
    numpy_to_transform,
    transform_to_numpy,
    numpy_from_translation_euler,
)


@pytest.fixture
def sample_pose():
    """Returns a sample Pose for testing."""
    pose = Pose()
    pose.position.x = 1.0
    pose.position.y = 2.0
    pose.position.z = 3.0
    pose.orientation.w = 1.0  # Identity quaternion
    return pose


@pytest.fixture
def sample_transform():
    """Returns a sample Transform for testing."""
    transform = Transform()
    transform.translation.x = 1.0
    transform.translation.y = 2.0
    transform.translation.z = 3.0
    transform.rotation.w = 1.0  # Identity quaternion
    return transform


def test_pose_to_posestamped(sample_pose):
    frame_id = "map"
    ps = pose_to_posestamped(sample_pose, frame_id)

    assert isinstance(ps, PoseStamped)
    assert ps.pose == sample_pose
    assert ps.header.frame_id == frame_id


def test_posestamped_to_pose(sample_pose):
    ps = PoseStamped(pose=sample_pose)
    pose = posestamped_to_pose(ps)

    assert isinstance(pose, Pose)
    assert pose == sample_pose


def test_pose_to_transform(sample_pose):
    transform = pose_to_transform(sample_pose)

    assert isinstance(transform, Transform)
    assert transform.translation.x == sample_pose.position.x
    assert transform.translation.y == sample_pose.position.y
    assert transform.translation.z == sample_pose.position.z
    assert transform.rotation.x == sample_pose.orientation.x
    assert transform.rotation.y == sample_pose.orientation.y
    assert transform.rotation.z == sample_pose.orientation.z
    assert transform.rotation.w == sample_pose.orientation.w


def test_transform_to_transformstamped(sample_transform):
    frame_id = "world"
    child_frame_id = "robot"
    ts = transform_to_transformstamped(
        sample_transform, frame_id, child_frame_id)

    assert isinstance(ts, TransformStamped)
    assert ts.transform == sample_transform
    assert ts.header.frame_id == frame_id
    assert ts.child_frame_id == child_frame_id


def test_transformstamped_to_transform(sample_transform):
    ts = TransformStamped(transform=sample_transform)
    transform = transformstamped_to_transform(ts)

    assert isinstance(transform, Transform)
    assert transform == sample_transform


def test_numpy_to_transform():
    matrix = np.eye(4)
    matrix[:3, 3] = [1.0, 2.0, 3.0]
    transform = numpy_to_transform(matrix)

    assert isinstance(transform, Transform)
    assert np.isclose(transform.translation.x, 1.0)
    assert np.isclose(transform.translation.y, 2.0)
    assert np.isclose(transform.translation.z, 3.0)


def test_transform_to_numpy(sample_transform):
    matrix = transform_to_numpy(sample_transform)

    assert isinstance(matrix, np.ndarray)
    assert matrix.shape == (4, 4)
    assert np.isclose(matrix[:3, 3], [1.0, 2.0, 3.0]).all()


def test_numpy_from_translation_euler():
    translation = np.array([1.0, 2.0, 3.0])
    euler_angles = np.array([0.1, 0.2, 0.3])
    matrix = numpy_from_translation_euler(translation, euler_angles)

    assert isinstance(matrix, np.ndarray)
    assert matrix.shape == (4, 4)
    assert np.isclose(matrix[:3, 3], translation).all()

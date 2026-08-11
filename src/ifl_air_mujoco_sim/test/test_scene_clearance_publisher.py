from types import SimpleNamespace

from builtin_interfaces.msg import Time
import numpy as np
from visualization_msgs.msg import Marker

from env import ros2_interface as ros2_module
from env.ros2_interface import UR10eRos2Interface
from env.utils.scene_clearance_bounds import SceneClearanceBound


class _FakeNow:
    def __init__(self, stamp):
        self._stamp = stamp

    def to_msg(self):
        return self._stamp


class _FakeClock:
    def __init__(self, stamp):
        self._stamp = stamp

    def now(self):
        return _FakeNow(self._stamp)


class _FakeNode:
    def __init__(self, stamp=None):
        self._clock = _FakeClock(stamp or Time(sec=123, nanosec=456))

    def get_clock(self):
        return self._clock


class _FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _FakeLogger:
    def __init__(self):
        self.errors = []
        self.infos = []

    def error(self, message, *args):
        self.errors.append(message % args if args else message)

    def info(self, message, *args):
        self.infos.append(message % args if args else message)


def _make_publish_interface(sim):
    interface = UR10eRos2Interface.__new__(UR10eRos2Interface)
    interface.sim = sim
    interface.node = _FakeNode()
    interface.scene_clearance_pub = _FakePublisher()
    interface.logger = _FakeLogger()
    interface._last_scene_clearance_error_log_time = float("-inf")
    interface._scene_clearance_profile_samples = []
    return interface


def test_scene_description_pose_text_and_frame_contract_is_unchanged():
    scene_object = {
        "name": "apple",
        "type": "mesh",
        "size": [],
        "position": [0.4, -0.2, 0.7],
        "orientation": [0.5, 0.1, 0.2, 0.3],
        "color": [1.0, 0.0, 0.0, 1.0],
    }
    interface = UR10eRos2Interface.__new__(UR10eRos2Interface)
    interface.node = _FakeNode()
    interface.sim = SimpleNamespace(get_scene_description=lambda: [scene_object])

    marker_array = interface._create_scene_marker_array()

    assert len(marker_array.markers) == 1
    marker = marker_array.markers[0]
    assert marker.header.frame_id == "base_link"
    assert marker.text == "apple"
    assert marker.ns == "scene_objects"
    assert marker.pose.position.x == 0.4
    assert marker.pose.position.y == -0.2
    assert marker.pose.position.z == 0.7
    assert marker.pose.orientation.w == 0.5
    assert marker.pose.orientation.x == 0.1
    assert marker.pose.orientation.y == 0.2
    assert marker.pose.orientation.z == 0.3


def test_clearance_bounds_builds_one_identity_cube_per_object_with_one_stamp():
    interface = UR10eRos2Interface.__new__(UR10eRos2Interface)
    bounds = [
        SceneClearanceBound(
            "apple",
            np.array([0.4, -0.2, 0.7]),
            np.array([0.075, 0.074, 0.072]),
        ),
        SceneClearanceBound(
            "hammer",
            np.array([-0.5, 0.3, 0.1]),
            np.array([0.18, 0.33, 0.04]),
        ),
    ]
    stamp = Time(sec=77, nanosec=88)

    marker_array = interface._create_scene_clearance_bounds_marker_array(bounds, stamp)

    assert len(marker_array.markers) == 2
    for index, (marker, bound) in enumerate(zip(marker_array.markers, bounds)):
        assert marker.header.stamp.sec == 77
        assert marker.header.stamp.nanosec == 88
        assert marker.header.frame_id == "base_link"
        assert marker.ns == "scene_clearance_bounds"
        assert marker.id == index
        assert marker.text == bound.name
        assert marker.type == Marker.CUBE
        assert marker.action == Marker.ADD
        assert marker.pose.orientation.x == 0.0
        assert marker.pose.orientation.y == 0.0
        assert marker.pose.orientation.z == 0.0
        assert marker.pose.orientation.w == 1.0
        np.testing.assert_allclose(
            [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z],
            bound.center,
        )
        np.testing.assert_allclose(
            [marker.scale.x, marker.scale.y, marker.scale.z],
            bound.size,
        )


def test_clearance_publisher_publishes_complete_array_only_after_success():
    bounds = [
        SceneClearanceBound("apple", np.array([0.0, 0.0, 0.1]), np.array([0.1, 0.1, 0.1])),
        SceneClearanceBound("hammer", np.array([0.2, 0.3, 0.1]), np.array([0.2, 0.3, 0.1])),
    ]
    interface = _make_publish_interface(
        SimpleNamespace(get_scene_clearance_bounds=lambda: bounds)
    )

    published = interface._publish_scene_clearance_bounds()

    assert published is True
    assert len(interface.scene_clearance_pub.messages) == 1
    assert [
        marker.text for marker in interface.scene_clearance_pub.messages[0].markers
    ] == ["apple", "hammer"]


def test_clearance_publisher_does_not_publish_when_one_bound_is_invalid():
    bounds = [
        SceneClearanceBound("apple", np.array([0.0, 0.0, 0.1]), np.array([0.1, 0.1, 0.1])),
        SceneClearanceBound("hammer", np.array([0.2, 0.3, 0.1]), np.array([0.2, 0.0, 0.1])),
    ]
    interface = _make_publish_interface(
        SimpleNamespace(get_scene_clearance_bounds=lambda: bounds)
    )

    published = interface._publish_scene_clearance_bounds()

    assert published is False
    assert interface.scene_clearance_pub.messages == []
    assert "not published" in interface.logger.errors[0]


def test_clearance_publisher_does_not_publish_when_sim_bound_computation_fails():
    def _raise():
        raise RuntimeError("hammer pose is invalid")

    interface = _make_publish_interface(
        SimpleNamespace(get_scene_clearance_bounds=_raise)
    )

    published = interface._publish_scene_clearance_bounds()

    assert published is False
    assert interface.scene_clearance_pub.messages == []
    assert "hammer pose is invalid" in interface.logger.errors[0]


def test_clearance_publisher_throttles_repeated_runtime_errors(monkeypatch):
    def _raise():
        raise RuntimeError("bad cache")

    interface = _make_publish_interface(
        SimpleNamespace(get_scene_clearance_bounds=_raise)
    )
    times = iter([100.0, 100.5, 102.1])
    monkeypatch.setattr(ros2_module.time, "monotonic", lambda: next(times))

    assert interface._publish_scene_clearance_bounds() is False
    assert interface._publish_scene_clearance_bounds() is False
    assert len(interface.logger.errors) == 1
    assert interface._publish_scene_clearance_bounds() is False
    assert len(interface.logger.errors) == 2


def test_clearance_publisher_reports_first_hundred_compute_samples_once():
    bounds = [
        SceneClearanceBound(
            "apple",
            np.array([0.0, 0.0, 0.1]),
            np.array([0.1, 0.1, 0.1]),
        )
    ]
    interface = _make_publish_interface(
        SimpleNamespace(get_scene_clearance_bounds=lambda: bounds)
    )

    for _ in range(101):
        assert interface._publish_scene_clearance_bounds() is True

    assert len(interface.logger.infos) == 1
    assert "samples=100" in interface.logger.infos[0]
    assert "p95=" in interface.logger.infos[0]
    assert interface._scene_clearance_profile_samples is None

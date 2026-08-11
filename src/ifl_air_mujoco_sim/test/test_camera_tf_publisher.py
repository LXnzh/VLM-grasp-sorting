import numpy as np

from env.ros2_interface import UR10eRos2Interface


def test_camera_tf_publishes_when_pointcloud_is_disabled():
    interface = UR10eRos2Interface.__new__(UR10eRos2Interface)
    interface.camera_1_name = "camera_orbbec"
    interface.camera_2_name = None
    interface._publish_pc_cam1 = False
    interface._publish_pc_cam2 = False
    extrinsics = np.eye(4, dtype=float)
    interface.sim = type(
        "FakeSim",
        (),
        {"get_camera_extrinsics": lambda _self, name: extrinsics.copy()},
    )()
    published = []
    interface._apply_pointcloud_transform = lambda value, _name: value
    interface._publish_camera_tf = (
        lambda name, value, sec, nsec: published.append(
            (name, value.copy(), sec, nsec)
        )
    )

    interface._publish_rendered_camera_transforms(12, 34)

    assert len(published) == 1
    name, value, sec, nsec = published[0]
    assert name == "camera_orbbec"
    np.testing.assert_allclose(value, extrinsics)
    assert (sec, nsec) == (12, 34)


def test_camera_tf_uses_the_existing_camera_frame_adjustment():
    interface = UR10eRos2Interface.__new__(UR10eRos2Interface)
    interface.camera_1_name = "camera_orbbec"
    interface.camera_2_name = None
    raw_extrinsics = np.eye(4, dtype=float)
    adjusted_extrinsics = np.eye(4, dtype=float)
    adjusted_extrinsics[0, 3] = 0.25
    interface.sim = type(
        "FakeSim",
        (),
        {"get_camera_extrinsics": lambda _self, name: raw_extrinsics.copy()},
    )()
    published = []
    interface._apply_pointcloud_transform = (
        lambda value, name: adjusted_extrinsics.copy()
    )
    interface._publish_camera_tf = (
        lambda name, value, sec, nsec: published.append((name, value))
    )

    interface._publish_rendered_camera_transforms(1, 2)

    assert published[0][0] == "camera_orbbec"
    np.testing.assert_allclose(published[0][1], adjusted_extrinsics)

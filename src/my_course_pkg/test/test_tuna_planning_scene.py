from types import SimpleNamespace

import numpy as np
import pytest

from my_course_pkg.grasp.tuna_planning_scene import table_top_z_from_planning_scene


def pose(x=0.0, y=0.0, z=0.0, quaternion=(0.0, 0.0, 0.0, 1.0)):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=z),
        orientation=SimpleNamespace(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3],
        ),
    )


def collision_object(
    object_id="Desk_1520x900_cell",
    frame="world",
    center_z=0.45,
    dimensions=(1.52, 0.9, 0.9),
    quaternion=(0.0, 0.0, 0.0, 1.0),
):
    return SimpleNamespace(
        id=object_id,
        header=SimpleNamespace(frame_id=frame),
        primitives=[SimpleNamespace(type=1, dimensions=list(dimensions))],
        primitive_poses=[pose(z=center_z, quaternion=quaternion)],
    )


def scene(*objects):
    return SimpleNamespace(world=SimpleNamespace(collision_objects=list(objects)))


def test_exact_cell_desk_top_is_read_from_live_box_geometry():
    assert table_top_z_from_planning_scene(scene(collision_object())) == pytest.approx(
        0.9
    )


def test_rotated_box_uses_world_projected_half_height():
    half_angle = np.pi / 4.0
    object_ = collision_object(
        center_z=1.0,
        dimensions=(0.2, 0.4, 0.6),
        quaternion=(0.0, np.sin(half_angle), 0.0, np.cos(half_angle)),
    )
    assert table_top_z_from_planning_scene(scene(object_)) == pytest.approx(1.1)


@pytest.mark.parametrize(
    "objects",
    [
        (),
        (collision_object(object_id="Desk_1520x900_hand"),),
        (collision_object(), collision_object()),
        (collision_object(frame="base_link"),),
    ],
)
def test_missing_duplicate_wrong_name_or_wrong_frame_fails_closed(objects):
    with pytest.raises(ValueError):
        table_top_z_from_planning_scene(scene(*objects))

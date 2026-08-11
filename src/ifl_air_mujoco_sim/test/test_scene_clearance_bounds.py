from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest
import yaml

from env import mjcontrol_interface as mjcontrol_module
from env.mjcontrol_interface import MuJoCoInterface
from env.utils import populate_scene
from env.utils.scene_clearance_bounds import (
    BoxGeometry,
    CylinderGeometry,
    MeshGeometry,
    ObjectGeometryCache,
    SphereGeometry,
    build_object_geometry_cache,
    compute_world_aabb,
    load_obj_vertices,
)
from env.utils.ycb_assets import resolve_ycb_assets


def _write_obj(path, vertices):
    lines = ["# synthetic mesh", "o fixture"]
    lines.extend("v " + " ".join(str(value) for value in vertex) for vertex in vertices)
    lines.append("f 1 2 3")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_ycb_dir(tmp_path, visual_vertices, collision_vertices):
    ycb_dir = tmp_path / "005_widget"
    ycb_dir.mkdir()
    _write_obj(ycb_dir / "textured.obj", visual_vertices)
    (ycb_dir / "textured.mtl").write_text(
        "newmtl material\nmap_Kd texture_map.png\n",
        encoding="utf-8",
    )
    (ycb_dir / "texture_map.png").write_bytes(b"fixture")
    for filename, vertices in collision_vertices.items():
        _write_obj(ycb_dir / filename, vertices)
    return ycb_dir


def _mesh_config(ycb_dir, name="widget"):
    return {
        "name": name,
        "type": "mesh",
        "ycb_dir": str(ycb_dir),
        "size": [],
    }


def _rotation_xyz(roll, pitch, yaw):
    cx, sx = np.cos(roll), np.sin(roll)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cz, sz = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=float)
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=float)
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def test_load_obj_vertices_reads_only_vertex_records(tmp_path):
    obj_path = tmp_path / "mesh.obj"
    obj_path.write_text(
        "# comment\n"
        "o object\n"
        "v 1 2 3\n"
        "vn 0 0 1\n"
        "vt 0.2 0.3\n"
        "v -4.5 5.5 6.5 1.0\n"
        "f 1 2 2\n",
        encoding="utf-8",
    )

    vertices = load_obj_vertices(obj_path)

    np.testing.assert_allclose(vertices, [[1.0, 2.0, 3.0], [-4.5, 5.5, 6.5]])
    assert vertices.dtype == np.float64


@pytest.mark.parametrize(
    "contents,match",
    [
        ("# no vertices\n", "no vertices"),
        ("v 1 2\n", "vertex record"),
        ("v nan 0 0\n", "non-finite"),
        ("v nope 0 0\n", "vertex record"),
    ],
)
def test_load_obj_vertices_rejects_empty_and_invalid_mesh(tmp_path, contents, match):
    obj_path = tmp_path / "invalid.obj"
    obj_path.write_text(contents, encoding="utf-8")

    with pytest.raises(ValueError, match=match):
        load_obj_vertices(obj_path)


def test_load_obj_vertices_rejects_missing_file(tmp_path):
    with pytest.raises(FileNotFoundError, match="missing.obj"):
        load_obj_vertices(tmp_path / "missing.obj")


def test_mesh_placement_info_preserves_raw_vertex_orientation_semantics(tmp_path):
    visual_vertices = np.array(
        [[1.0, 0.0, -1.0], [3.0, 2.0, 1.0], [2.0, -1.0, 0.0]],
        dtype=float,
    )
    collision_vertices_0 = np.array(
        [[-2.0, 0.0, -3.0], [0.0, 1.0, 0.5], [1.0, -2.0, 1.0]],
        dtype=float,
    )
    collision_vertices_1 = np.array(
        [[0.0, 2.0, -2.0], [2.0, 0.0, 2.0], [-1.0, -1.0, 0.0]],
        dtype=float,
    )
    ycb_dir = _write_ycb_dir(
        tmp_path,
        visual_vertices,
        {
            "textured_vhacd_collision_0.obj": collision_vertices_0,
            "textured_vhacd_collision_1.obj": collision_vertices_1,
        },
    )
    orientation = [0.35, -0.2, np.pi / 3.0]
    rotation = _rotation_xyz(*orientation)
    rotated_visual = (rotation @ visual_vertices.T).T
    rotated_collision = np.concatenate(
        [
            (rotation @ collision_vertices_0.T).T,
            (rotation @ collision_vertices_1.T).T,
        ]
    )
    assets = resolve_ycb_assets("widget", ycb_dir)

    placement = populate_scene._compute_mesh_placement_info(
        assets,
        orientation=orientation,
    )

    assert placement["collision_z_min"] == pytest.approx(
        rotated_collision[:, 2].min()
    )
    assert placement["centroid_x"] == pytest.approx(rotated_visual[:, 0].mean())
    assert placement["centroid_y"] == pytest.approx(rotated_visual[:, 1].mean())


def test_mesh_cache_unions_visual_and_collision_geometries(tmp_path):
    ycb_dir = _write_ycb_dir(
        tmp_path,
        [[0, 0, 0], [1, 1, 1], [0.5, 0.5, 0.5]],
        {
            "textured_vhacd_collision_0.obj": [
                [-2, 0, 0],
                [0, 3, 0],
                [0, 0, 4],
            ],
        },
    )

    cache = build_object_geometry_cache(_mesh_config(ycb_dir))
    bound = compute_world_aabb(cache, np.zeros(3), np.eye(3))

    assert len(cache.geometries) == 2
    np.testing.assert_allclose(bound.center, [-0.5, 1.5, 2.0])
    np.testing.assert_allclose(bound.size, [3.0, 3.0, 4.0])


def test_mesh_world_aabb_matches_bruteforce_for_multiple_rotations():
    vertices = np.array(
        [[-1.0, -0.5, 0.0], [2.0, 0.25, 1.5], [0.5, 3.0, -2.0]],
        dtype=float,
    )
    cache = ObjectGeometryCache("mesh", (MeshGeometry(vertices),))
    position = np.array([0.4, -1.2, 2.5], dtype=float)

    for rotation in (
        np.eye(3),
        _rotation_xyz(0.0, 0.0, np.pi / 2.0),
        _rotation_xyz(0.31, -0.47, 1.17),
    ):
        bound = compute_world_aabb(cache, position, rotation)
        world_vertices = (rotation @ vertices.T).T + position
        minimum = world_vertices.min(axis=0)
        maximum = world_vertices.max(axis=0)
        np.testing.assert_allclose(bound.center, 0.5 * (minimum + maximum))
        np.testing.assert_allclose(bound.size, maximum - minimum)


def test_off_origin_mesh_reports_geometry_center_not_body_origin():
    vertices = np.array([[1, 2, 3], [3, 6, 9], [2, 4, 5]], dtype=float)
    cache = ObjectGeometryCache("offset", (MeshGeometry(vertices),))

    bound = compute_world_aabb(cache, np.zeros(3), np.eye(3))

    np.testing.assert_allclose(bound.center, [2.0, 4.0, 6.0])
    assert not np.allclose(bound.center, np.zeros(3))


def test_box_world_extent_uses_absolute_rotation_projection():
    rotation = _rotation_xyz(0.41, -0.22, 0.73)
    half_extents = np.array([0.2, 0.4, 0.8], dtype=float)
    geometry = BoxGeometry(np.array([0.1, -0.2, 0.3]), np.eye(3), half_extents)
    cache = ObjectGeometryCache("box", (geometry,))

    bound = compute_world_aabb(cache, np.array([1.0, 2.0, 3.0]), rotation)

    np.testing.assert_allclose(bound.center, rotation @ geometry.center_body + [1, 2, 3])
    np.testing.assert_allclose(bound.size, 2.0 * (np.abs(rotation) @ half_extents))


def test_sphere_world_extent_is_rotation_invariant():
    geometry = SphereGeometry(np.array([0.1, 0.2, 0.3]), 0.7)
    cache = ObjectGeometryCache("sphere", (geometry,))
    rotation = _rotation_xyz(0.6, 0.2, -1.0)

    bound = compute_world_aabb(cache, np.array([1.0, 2.0, 3.0]), rotation)

    np.testing.assert_allclose(bound.center, rotation @ geometry.center_body + [1, 2, 3])
    np.testing.assert_allclose(bound.size, [1.4, 1.4, 1.4])


def test_cylinder_world_extent_uses_axis_and_radial_projection():
    rotation = _rotation_xyz(0.7, -0.4, 0.2)
    geometry = CylinderGeometry(np.zeros(3), np.eye(3), radius=0.3, half_height=0.8)
    cache = ObjectGeometryCache("cylinder", (geometry,))

    bound = compute_world_aabb(cache, np.zeros(3), rotation)

    axis = rotation[:, 2]
    expected_half = 0.8 * np.abs(axis) + 0.3 * np.sqrt(np.maximum(0.0, 1.0 - axis**2))
    np.testing.assert_allclose(bound.size, 2.0 * expected_half)


@pytest.mark.parametrize(
    "position,rotation",
    [
        ([np.nan, 0, 0], np.eye(3)),
        ([0, 0, 0], np.full((3, 3), np.inf)),
        ([0, 0, 0], np.zeros((2, 2))),
    ],
)
def test_world_aabb_rejects_nonfinite_or_invalid_pose(position, rotation):
    cache = ObjectGeometryCache(
        "mesh",
        (MeshGeometry(np.array([[0, 0, 0], [1, 1, 1]], dtype=float)),),
    )

    with pytest.raises(ValueError, match="mesh"):
        compute_world_aabb(cache, position, rotation)


def test_world_aabb_rejects_nonpositive_extent():
    cache = ObjectGeometryCache(
        "flat",
        (MeshGeometry(np.array([[0, 0, 0], [1, 1, 0]], dtype=float)),),
    )

    with pytest.raises(ValueError, match="extent"):
        compute_world_aabb(cache, np.zeros(3), np.eye(3))


@pytest.mark.parametrize(
    "config",
    [
        {"name": "box", "type": "box", "size": [0.1, 0.2, 0.3]},
        {"name": "sphere", "type": "sphere", "size": [0.2]},
        {"name": "cylinder", "type": "cylinder", "size": [0.2, 0.4]},
    ],
)
def test_primitive_config_builds_positive_geometry_cache(config):
    cache = build_object_geometry_cache(config)

    assert cache.object_name == config["name"]
    assert len(cache.geometries) == 1


@pytest.mark.parametrize(
    "config",
    [
        {"name": "box", "type": "box", "size": [0.1, 0.0, 0.3]},
        {"name": "sphere", "type": "sphere", "size": [np.nan]},
        {"name": "cylinder", "type": "cylinder", "size": [0.2]},
        {"name": "capsule", "type": "capsule", "size": [0.2, 0.4]},
    ],
)
def test_primitive_config_rejects_invalid_or_unsupported_geometry(config):
    with pytest.raises(ValueError, match=config["name"]):
        build_object_geometry_cache(config)


def _real_ycb_config(name):
    config_path = Path(__file__).resolve().parents[1] / "env/config/base_env.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    return next(obj for obj in config["objects"] if obj["name"] == name)


@pytest.mark.parametrize("name", ["apple", "banana", "hammer"])
def test_real_ycb_cache_contains_visual_and_collision_meshes(name):
    cache = build_object_geometry_cache(_real_ycb_config(name))

    assert len(cache.geometries) >= 2
    assert all(isinstance(geometry, MeshGeometry) for geometry in cache.geometries)


def test_real_apple_identity_bound_matches_reference_size():
    cache = build_object_geometry_cache(_real_ycb_config("apple"))

    bound = compute_world_aabb(cache, np.zeros(3), np.eye(3))

    np.testing.assert_allclose(
        bound.size,
        [0.075448, 0.074871, 0.071889],
        atol=2e-3,
    )


@pytest.mark.parametrize("name", ["banana", "hammer"])
def test_long_real_ycb_bounds_are_not_generic_point_one_meter_cubes(name):
    cache = build_object_geometry_cache(_real_ycb_config(name))

    bound = compute_world_aabb(cache, np.zeros(3), np.eye(3))

    assert not np.allclose(bound.size, [0.1, 0.1, 0.1], atol=1e-6)
    assert max(bound.size[:2]) > 0.15


def test_real_hammer_bound_center_is_offset_from_body_origin():
    cache = build_object_geometry_cache(_real_ycb_config("hammer"))

    bound = compute_world_aabb(cache, np.zeros(3), np.eye(3))

    assert np.linalg.norm(bound.center) > 0.02


def _real_hammer_spawn_root():
    config = _real_ycb_config("hammer")
    assets = resolve_ycb_assets(config["name"], config["ycb_dir"])
    orientation = config.get("orientation", [0.0, 0.0, 0.0])
    placement = populate_scene._compute_mesh_placement_info(
        assets,
        orientation=orientation,
    )
    body_z = (
        -placement["collision_z_min"]
        + populate_scene.MESH_SPAWN_CLEARANCE_M
    )
    root = populate_scene.build_ycb_object_xml("hammer", assets)
    body = root.find("./worldbody/body[@name='hammer']")
    body.set("pos", f"0 0 {body_z}")
    body.set("euler", " ".join(str(value) for value in orientation))
    ET.SubElement(
        root.find("worldbody"),
        "geom",
        name="table",
        type="plane",
        pos="0 0 0",
        size="1 1 0.1",
        friction="0.9 0.2 0.05",
    )
    ET.SubElement(root, "option", timestep="0.002", gravity="0 0 -9.81")
    return root, body_z, placement


def test_real_hammer_collision_bottom_spawns_with_half_millimetre_clearance():
    _, body_z, placement = _real_hammer_spawn_root()

    collision_bottom = body_z + placement["collision_z_min"]

    assert collision_bottom == pytest.approx(
        populate_scene.MESH_SPAWN_CLEARANCE_M,
        abs=1e-12,
    )


def test_real_hammer_settles_without_launching_from_table():
    root, _, _ = _real_hammer_spawn_root()
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hammer")
    mujoco.mj_forward(model, data)
    initial_position = data.xpos[body_id].copy()
    max_upward_displacement = 0.0
    max_horizontal_displacement = 0.0

    for _ in range(500):
        mujoco.mj_step(model, data)
        displacement = data.xpos[body_id] - initial_position
        max_upward_displacement = max(
            max_upward_displacement,
            float(displacement[2]),
        )
        max_horizontal_displacement = max(
            max_horizontal_displacement,
            float(np.linalg.norm(displacement[:2])),
        )

    assert max_upward_displacement <= 0.005
    assert max_horizontal_displacement <= 0.005


def _make_interface_with_geometry_entries(entries):
    interface = MuJoCoInterface.__new__(MuJoCoInterface)
    interface.objects_config = [{"name": name} for name, _, _ in entries]
    interface._scene_clearance_geometry_entries = tuple(entries)
    body_count = max(body_id for _, body_id, _ in entries) + 1
    interface.data = SimpleNamespace(
        xpos=np.zeros((body_count, 3), dtype=float),
        xmat=np.repeat(np.eye(3, dtype=float).reshape(1, 9), body_count, axis=0),
    )
    return interface


def test_mujoco_interface_returns_all_bounds_in_selected_order():
    apple_cache = ObjectGeometryCache(
        "apple",
        (MeshGeometry(np.array([[0, 0, 0], [1, 2, 3]], dtype=float)),),
    )
    hammer_cache = ObjectGeometryCache(
        "hammer",
        (MeshGeometry(np.array([[-2, -1, -0.5], [2, 1, 0.5]], dtype=float)),),
    )
    interface = _make_interface_with_geometry_entries(
        [("apple", 0, apple_cache), ("hammer", 1, hammer_cache)]
    )
    interface.data.xpos[0] = [10, 20, 30]
    interface.data.xpos[1] = [-1, -2, -3]

    bounds = interface.get_scene_clearance_bounds()

    assert [bound.name for bound in bounds] == ["apple", "hammer"]
    np.testing.assert_allclose(bounds[0].center, [10.5, 21.0, 31.5])
    np.testing.assert_allclose(bounds[0].size, [1.0, 2.0, 3.0])
    np.testing.assert_allclose(bounds[1].center, [-1.0, -2.0, -3.0])


def test_mujoco_interface_uses_live_body_pose_each_call():
    cache = ObjectGeometryCache(
        "apple",
        (MeshGeometry(np.array([[0, 0, 0], [1, 2, 3]], dtype=float)),),
    )
    interface = _make_interface_with_geometry_entries([("apple", 0, cache)])

    first = interface.get_scene_clearance_bounds()[0]
    interface.data.xpos[0] = [0.4, -0.2, 0.8]
    interface.data.xmat[0] = _rotation_xyz(0.0, 0.0, np.pi / 2.0).reshape(9)
    second = interface.get_scene_clearance_bounds()[0]

    assert not np.allclose(first.center, second.center)
    np.testing.assert_allclose(second.size, [2.0, 1.0, 3.0], atol=1e-12)


def test_mujoco_interface_rejects_incomplete_geometry_entry_cache():
    cache = ObjectGeometryCache(
        "apple",
        (MeshGeometry(np.array([[0, 0, 0], [1, 2, 3]], dtype=float)),),
    )
    interface = _make_interface_with_geometry_entries([("apple", 0, cache)])
    interface.objects_config.append({"name": "hammer"})

    with pytest.raises(RuntimeError, match="geometry cache"):
        interface.get_scene_clearance_bounds()


def test_mujoco_interface_raises_without_returning_partial_runtime_bounds():
    apple_cache = ObjectGeometryCache(
        "apple",
        (MeshGeometry(np.array([[0, 0, 0], [1, 2, 3]], dtype=float)),),
    )
    invalid_hammer_cache = ObjectGeometryCache(
        "hammer",
        (MeshGeometry(np.array([[0, 0, 0], [1, 1, 0]], dtype=float)),),
    )
    interface = _make_interface_with_geometry_entries(
        [("apple", 0, apple_cache), ("hammer", 1, invalid_hammer_cache)]
    )

    with pytest.raises(RuntimeError, match="hammer"):
        interface.get_scene_clearance_bounds()


def test_mujoco_interface_initializes_cache_and_body_ids_in_selection_order(monkeypatch):
    interface = MuJoCoInterface.__new__(MuJoCoInterface)
    interface.objects_config = [
        {"name": "apple", "type": "sphere", "size": [0.1]},
        {"name": "hammer", "type": "box", "size": [0.1, 0.2, 0.3]},
    ]
    interface.model = object()
    resolved_ids = {"apple": 7, "hammer": 9}
    monkeypatch.setattr(
        mjcontrol_module.mujoco,
        "mj_name2id",
        lambda model, object_type, name: resolved_ids[name],
    )

    interface._initialize_scene_clearance_geometry_cache()

    assert [
        (name, body_id)
        for name, body_id, _ in interface._scene_clearance_geometry_entries
    ] == [("apple", 7), ("hammer", 9)]


def test_mujoco_interface_rejects_missing_or_duplicate_bodies(monkeypatch):
    interface = MuJoCoInterface.__new__(MuJoCoInterface)
    interface.model = object()
    interface.objects_config = [
        {"name": "apple", "type": "sphere", "size": [0.1]},
        {"name": "apple", "type": "sphere", "size": [0.1]},
    ]
    monkeypatch.setattr(mjcontrol_module.mujoco, "mj_name2id", lambda *args: 3)
    with pytest.raises(RuntimeError, match="Duplicate"):
        interface._initialize_scene_clearance_geometry_cache()

    interface.objects_config = [
        {"name": "hammer", "type": "box", "size": [0.1, 0.2, 0.3]},
    ]
    monkeypatch.setattr(mjcontrol_module.mujoco, "mj_name2id", lambda *args: -1)
    with pytest.raises(RuntimeError, match="hammer"):
        interface._initialize_scene_clearance_geometry_cache()

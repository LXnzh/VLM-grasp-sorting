from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from env.utils.populate_scene import build_ycb_object_xml
from env.utils.ycb_assets import resolve_ycb_assets


EXPECTED_SCENE_YCB_DIRS = {
    "tomato_soup_can": "005_tomato_soup_can",
    "gelatin_box": "009_gelatin_box",
    "banana": "011_banana",
    "apple": "013_apple",
    "lemon": "014_lemon",
    "peach": "015_peach",
    "pear": "016_pear",
    "orange": "017_orange",
    "plum": "018_plum",
    "sponge": "026_sponge",
    "hammer": "048_hammer",
    "baseball": "055_baseball",
    "tennis_ball": "056_tennis_ball",
    "racquetball": "057_racquetball",
    "foam_brick": "061_foam_brick",
    "rubiks_cube": "077_rubiks_cube",
}


def _write_ycb_dir(tmp_path):
    ycb_dir = tmp_path / "005_widget"
    ycb_dir.mkdir()
    (ycb_dir / "textured.obj").write_text(
        "v 0 0 0\nv 1 1 1\nv -1 -1 -1\nf 1 2 3\n",
        encoding="utf-8",
    )
    (ycb_dir / "textured.mtl").write_text(
        "newmtl material\nmap_Kd texture_map.png\n",
        encoding="utf-8",
    )
    (ycb_dir / "texture_map.png").write_bytes(b"fixture")
    for suffix in ("0", "10", "1"):
        (ycb_dir / f"textured_vhacd_collision_{suffix}.obj").write_text(
            "v 0 0 0\nv 1 1 1\nv -1 -1 -1\nf 1 2 3\n",
            encoding="utf-8",
        )
    return ycb_dir


def test_numbered_ycb_assets_are_validated_and_sorted(tmp_path):
    ycb_dir = _write_ycb_dir(tmp_path)

    assets = resolve_ycb_assets("widget", ycb_dir)

    assert assets.directory == ycb_dir
    assert assets.visual_mesh.name == "textured.obj"
    assert assets.material.name == "textured.mtl"
    assert assets.texture.name == "texture_map.png"
    assert [path.name for path in assets.collision_meshes] == [
        "textured_vhacd_collision_0.obj",
        "textured_vhacd_collision_1.obj",
        "textured_vhacd_collision_10.obj",
    ]


@pytest.mark.parametrize(
    "missing_name,match",
    [
        ("textured.obj", "visual mesh.*textured.obj"),
        ("textured.mtl", "material.*textured.mtl"),
        ("texture_map.png", "texture.*texture_map.png"),
        ("collisions", r"collision meshes.*textured_vhacd_collision_\*\.obj"),
    ],
)
def test_numbered_ycb_assets_fail_clearly_when_required_data_is_missing(
    tmp_path,
    missing_name,
    match,
):
    ycb_dir = _write_ycb_dir(tmp_path)
    if missing_name == "collisions":
        for path in ycb_dir.glob("textured_vhacd_collision_*.obj"):
            path.unlink()
    else:
        (ycb_dir / missing_name).unlink()

    with pytest.raises(FileNotFoundError, match=match) as error:
        resolve_ycb_assets("widget", ycb_dir)

    assert "widget" in str(error.value)
    assert str(ycb_dir) in str(error.value)


def test_numbered_ycb_assets_reject_missing_directory(tmp_path):
    ycb_dir = tmp_path / "005_widget"

    with pytest.raises(FileNotFoundError, match="widget.*005_widget"):
        resolve_ycb_assets("widget", ycb_dir)


def test_ycb_builder_preserves_visual_and_collision_contract(tmp_path):
    assets = resolve_ycb_assets("widget", _write_ycb_dir(tmp_path))

    root = build_ycb_object_xml("widget", assets)

    mesh_elements = root.findall("asset/mesh")
    assert [Path(element.get("file")).name for element in mesh_elements] == [
        "textured.obj",
        "textured_vhacd_collision_0.obj",
        "textured_vhacd_collision_1.obj",
        "textured_vhacd_collision_10.obj",
    ]
    body = root.find("worldbody/body")
    assert body is not None
    assert body.get("name") == "widget"
    assert body.find("joint").attrib == {
        "name": "widget_joint",
        "type": "free",
    }
    geoms = body.findall("geom")
    assert geoms[0].attrib == {
        "name": "widget_visual",
        "type": "mesh",
        "mesh": "widget_visual_mesh",
        "material": "widget_mat",
        "contype": "0",
        "conaffinity": "0",
        "group": "2",
    }
    assert geoms[1].get("mass") == "0.1"
    assert all(geom.get("friction") == "0.9 0.2 0.05" for geom in geoms[1:])
    assert all(geom.get("group") == "3" for geom in geoms[1:])
    assert all(geom.get("mass") is None for geom in geoms[2:])


def test_default_scene_config_uses_sixteen_numbered_ycb_objects():
    config_path = Path(__file__).resolve().parents[1] / "env/config/base_env.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    by_name = {obj["name"]: obj for obj in config["objects"]}

    assert set(by_name) == set(EXPECTED_SCENE_YCB_DIRS)
    assert len({obj["ycb_dir"] for obj in by_name.values()}) == 16
    for name, directory_name in EXPECTED_SCENE_YCB_DIRS.items():
        object_config = by_name[name]
        assert "xml_path" not in object_config
        assert Path(object_config["ycb_dir"]).name == directory_name
        resolve_ycb_assets(name, object_config["ycb_dir"])


def test_generated_ycb_fragment_is_valid_xml(tmp_path):
    assets = resolve_ycb_assets("widget", _write_ycb_dir(tmp_path))

    xml_text = ET.tostring(
        build_ycb_object_xml("widget", assets),
        encoding="unicode",
    )

    parsed = ET.fromstring(xml_text)
    assert parsed.find("worldbody/body[@name='widget']") is not None

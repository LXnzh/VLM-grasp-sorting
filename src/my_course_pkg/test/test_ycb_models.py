from pathlib import Path

import pytest

from my_course_pkg.ycb_models import (
    NAME2MESH,
    YCB_DIRECTORY_BY_OBJECT,
    canonical_ycb_object_name,
    numbered_ycb_directory_name,
    resolve_ycb_visual_assets,
)


def _write_visual_assets(mesh_root, directory_name, texture_name="texture_map.png"):
    directory = mesh_root / directory_name
    directory.mkdir(parents=True)
    (directory / "textured.obj").write_text(
        "v 0 0 0\nv 1 1 1\nv -1 -1 -1\nf 1 2 3\n",
        encoding="utf-8",
    )
    (directory / "textured.mtl").write_text(
        f"newmtl material\nmap_Kd {texture_name}\n",
        encoding="utf-8",
    )
    (directory / "texture_map.png").write_bytes(b"standard")
    if texture_name != "texture_map.png":
        (directory / texture_name).write_bytes(b"referenced")
    return directory


@pytest.mark.parametrize(
    "object_name,directory_name",
    sorted(YCB_DIRECTORY_BY_OBJECT.items()),
)
def test_supported_space_and_underscore_aliases_resolve_to_numbered_directory(
    object_name,
    directory_name,
):
    space_alias = object_name.replace("_", " ")

    assert canonical_ycb_object_name(space_alias) == object_name
    assert numbered_ycb_directory_name(object_name) == directory_name
    assert numbered_ycb_directory_name(space_alias) == directory_name
    assert NAME2MESH[object_name] == directory_name
    assert NAME2MESH[space_alias] == directory_name


def test_unknown_ycb_object_name_fails_closed():
    with pytest.raises(ValueError, match="Unsupported YCB object name.*mystery"):
        numbered_ycb_directory_name("mystery object")


def test_visual_asset_resolution_uses_exact_numbered_directory(tmp_path):
    directory = _write_visual_assets(
        tmp_path,
        "005_tomato_soup_can",
        texture_name="label.png",
    )

    assets = resolve_ycb_visual_assets(tmp_path, "tomato soup can")

    assert assets.object_name == "tomato_soup_can"
    assert assets.directory == directory
    assert assets.mesh == directory / "textured.obj"
    assert assets.material == directory / "textured.mtl"
    assert assets.texture == directory / "label.png"
    assert assets.texture_archive_name == "label.png"


@pytest.mark.parametrize(
    "missing_name,match",
    [
        ("textured.obj", "mesh.*textured.obj"),
        ("textured.mtl", "material.*textured.mtl"),
        ("texture_map.png", "texture.*texture_map.png"),
    ],
)
def test_visual_asset_resolution_reports_missing_required_file(
    tmp_path,
    missing_name,
    match,
):
    directory = _write_visual_assets(tmp_path, "013_apple")
    (directory / missing_name).unlink()

    with pytest.raises(FileNotFoundError, match=match) as error:
        resolve_ycb_visual_assets(tmp_path, "apple")

    assert "apple" in str(error.value)
    assert str(directory) in str(error.value)


def test_visual_asset_resolution_reports_missing_material_texture(tmp_path):
    directory = _write_visual_assets(tmp_path, "013_apple")
    (directory / "textured.mtl").write_text(
        "newmtl material\nmap_Kd missing.png\n",
        encoding="utf-8",
    )

    with pytest.raises(FileNotFoundError, match="missing.png"):
        resolve_ycb_visual_assets(tmp_path, "apple")


def test_visual_asset_resolution_requires_numbered_directory(tmp_path):
    expected = Path(tmp_path) / "048_hammer"

    with pytest.raises(FileNotFoundError, match="hammer.*048_hammer"):
        resolve_ycb_visual_assets(tmp_path, "hammer")

"""Canonical mapping and visual assets for the supported YCB object set."""

from dataclasses import dataclass
from pathlib import Path


YCB_DIRECTORY_BY_OBJECT = {
    "tomato_soup_can": "005_tomato_soup_can",
    "tuna_fish_can": "007_tuna_fish_can",
    "pudding_box": "008_pudding_box",
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

NAME2MESH = {
    alias: directory
    for object_name, directory in YCB_DIRECTORY_BY_OBJECT.items()
    for alias in (object_name, object_name.replace("_", " "))
}


@dataclass(frozen=True)
class YcbVisualAssets:
    """FoundationPose bundle assets for one supported YCB object."""

    object_name: str
    directory: Path
    mesh: Path
    material: Path
    texture: Path
    texture_archive_name: str


def canonical_ycb_object_name(value):
    """Normalize supported underscore/space object aliases."""
    return "_".join(str(value or "").strip().lower().replace("_", " ").split())


def numbered_ycb_directory_name(value):
    """Resolve a supported friendly object name to its numbered directory."""
    object_name = canonical_ycb_object_name(value)
    try:
        return YCB_DIRECTORY_BY_OBJECT[object_name]
    except KeyError as exc:
        raise ValueError(f"Unsupported YCB object name: {value!r}") from exc


def resolve_ycb_visual_assets(mesh_root, value):
    """Validate exact FoundationPose assets for one supported object."""
    object_name = canonical_ycb_object_name(value)
    directory = Path(mesh_root) / numbered_ycb_directory_name(object_name)
    if not directory.is_dir():
        raise FileNotFoundError(
            f"YCB directory not found for object {object_name!r}: {directory}"
        )

    mesh = directory / "textured.obj"
    material = directory / "textured.mtl"
    standard_texture = directory / "texture_map.png"
    for description, path in (
        ("mesh", mesh),
        ("material", material),
        ("texture", standard_texture),
    ):
        if not path.is_file():
            raise FileNotFoundError(
                f"Missing YCB {description} for object {object_name!r} in "
                f"{directory}: {path.name}"
            )

    texture_lines = [
        line.split()
        for line in material.read_text(
            encoding="utf-8",
            errors="ignore",
        ).splitlines()
        if line.strip().startswith("map_Kd")
    ]
    if not texture_lines or len(texture_lines[0]) < 2:
        raise ValueError(
            f"YCB material for object {object_name!r} in {directory} "
            "does not define map_Kd"
        )
    texture_archive_name = texture_lines[0][-1]
    texture = directory / texture_archive_name
    if not texture.is_file():
        raise FileNotFoundError(
            f"Missing YCB material texture for object {object_name!r} in "
            f"{directory}: {texture_archive_name}"
        )

    return YcbVisualAssets(
        object_name=object_name,
        directory=directory,
        mesh=mesh,
        material=material,
        texture=texture,
        texture_archive_name=texture_archive_name,
    )

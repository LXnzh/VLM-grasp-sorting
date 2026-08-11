"""Validation for one numbered YCB object directory."""

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class YcbAssetPaths:
    """Required visual and collision assets for one YCB object."""

    directory: Path
    visual_mesh: Path
    material: Path
    texture: Path
    collision_meshes: tuple


def resolve_ycb_assets(object_name, ycb_dir):
    """Return validated paths for one configured numbered YCB directory."""
    name = str(object_name or "").strip()
    if not name:
        raise ValueError("YCB asset resolution requires a non-empty object name")
    if not ycb_dir:
        raise ValueError(f"Mesh object {name!r} requires ycb_dir")

    directory = Path(str(ycb_dir))
    if not directory.is_dir():
        raise FileNotFoundError(
            f"YCB directory not found for object {name!r}: {directory}"
        )

    required = {
        "visual mesh": directory / "textured.obj",
        "material": directory / "textured.mtl",
        "texture": directory / "texture_map.png",
    }
    for description, path in required.items():
        if not path.is_file():
            raise FileNotFoundError(
                f"Missing YCB {description} for object {name!r} in "
                f"{directory}: {path.name}"
            )

    collision_meshes = tuple(
        sorted(
            directory.glob("textured_vhacd_collision_*.obj"),
            key=lambda path: path.name,
        )
    )
    if not collision_meshes:
        raise FileNotFoundError(
            f"Missing YCB collision meshes for object {name!r} in "
            f"{directory}: textured_vhacd_collision_*.obj"
        )

    return YcbAssetPaths(
        directory=directory,
        visual_mesh=required["visual mesh"],
        material=required["material"],
        texture=required["texture"],
        collision_meshes=collision_meshes,
    )

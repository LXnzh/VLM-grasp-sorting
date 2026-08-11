"""FoundationPose mesh discovery and request-bundle construction."""

import io
import zipfile
from collections.abc import Callable
from pathlib import Path

import numpy as np
from PIL import Image


NAME2MESH = {
    "tomato soup can": "tomato_soup_can",
    "tuna fish can": "tuna_fish_can",
    "pudding box": "pudding_box",
    "gelatin box": "gelatin_box",
    "banana": "banana",
    "apple": "apple",
    "lemon": "lemon",
    "peach": "peach",
    "pear": "pear",
    "orange": "orange",
    "plum": "plum",
    "sponge": "sponge",
    "hammer": "hammer",
    "baseball": "baseball",
    "tennis ball": "tennis_ball",
    "racquetball": "racquetball",
    "foam brick": "foam_brick",
    "rubiks cube": "rubiks_cube",
}


def mesh_dir(
    mesh_root: Path,
    target: str,
    *,
    name_to_mesh: dict[str, str] = NAME2MESH,
) -> Path:
    mesh_name = name_to_mesh.get(target, target.replace(" ", "_"))
    target_mesh_dir = mesh_root / mesh_name
    if target_mesh_dir.is_dir():
        return target_mesh_dir

    ycb_dirs = sorted(
        path
        for path in mesh_root.iterdir()
        if path.is_dir() and path.name.endswith(f"_{mesh_name}")
    )
    if ycb_dirs:
        return ycb_dirs[0]

    available = sorted(path.name for path in mesh_root.iterdir() if path.is_dir())
    raise FileNotFoundError(
        f"Could not find mesh directory for {target!r}: tried {target_mesh_dir} "
        f"and '*_{mesh_name}' under {mesh_root}. "
        f"Available mesh directories: {available}"
    )


def build_bundle(
    *,
    rgb_path: Path,
    depth_path: Path,
    target: str,
    mask: np.ndarray,
    mesh_dir_for_target: Callable[[str], Path],
    camera_matrix_for_frame: Callable[[int, int], np.ndarray],
) -> bytes:
    """Build the exact zip payload consumed by the FoundationPose service."""
    rgb = np.array(Image.open(rgb_path).convert("RGB")).astype(np.uint8)
    depth = np.load(depth_path).astype(np.float32)
    if depth.shape != rgb.shape[:2] or mask.shape != rgb.shape[:2]:
        raise RuntimeError(
            "FoundationPose RGB/depth/mask size mismatch: "
            f"rgb={rgb.shape[:2]}, depth={depth.shape}, mask={mask.shape}."
        )
    frame_k = camera_matrix_for_frame(rgb.shape[1], rgb.shape[0])

    target_mesh_dir = mesh_dir_for_target(target)
    obj = next(target_mesh_dir.rglob("textured.obj"))
    mtl = next(target_mesh_dir.rglob("textured.mtl"))
    tex_name = next(
        line.split()[-1]
        for line in mtl.read_text(errors="ignore").splitlines()
        if line.strip().startswith("map_Kd")
    )
    tex = next(target_mesh_dir.rglob(tex_name))

    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w", zipfile.ZIP_DEFLATED) as archive:
        for name, array in [
            ("rgb.npy", rgb),
            ("depth.npy", depth),
            ("mask.npy", mask),
            ("cam_K.npy", frame_k),
        ]:
            array_buffer = io.BytesIO()
            np.save(array_buffer, array)
            archive.writestr(name, array_buffer.getvalue())

        archive.write(obj, "mesh/textured.obj")
        archive.write(mtl, "mesh/textured.mtl")
        archive.write(tex, f"mesh/{tex_name}")

    return buffer.getvalue()

#!/usr/bin/env python3
"""Render a FoundationPose mesh projection over the selected SAM2 mask.

This is an offline diagnostic tool, not part of the installed ROS package.
It accepts a saved pose matrix or, when omitted, runs the package's
FoundationPose client and parses its printed matrix.
"""

import argparse
import json
import math
import re
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from pycocotools import mask as mask_utils


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = REPO_ROOT / "my_course_pkg" / "outputs_local"
DEFAULT_SELECTED_JSON = DEFAULT_OUTPUT_DIR / "vlm_sam2_test" / "selected_object.json"
DEFAULT_RESPONSE_JSON = DEFAULT_OUTPUT_DIR / "vlm_sam2_test" / "response.json"
DEFAULT_RGB_PATH = DEFAULT_OUTPUT_DIR / "rgbd_frame" / "rgb.png"
DEFAULT_MESH_ROOT = REPO_ROOT / "YCB_Dataset" / "ycb"
DEFAULT_OVERLAY_PATH = DEFAULT_OUTPUT_DIR / "foundationpose" / "verification_overlay.png"
NAME2MESH = {
    "tomato soup can": "005_tomato_soup_can",
    "tomato_soup_can": "005_tomato_soup_can",
    "tuna fish can": "007_tuna_fish_can",
    "tuna_fish_can": "007_tuna_fish_can",
    "pudding box": "008_pudding_box",
    "pudding_box": "008_pudding_box",
    "gelatin box": "009_gelatin_box",
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
    "tennis ball": "056_tennis_ball",
    "tennis_ball": "056_tennis_ball",
    "racquetball": "057_racquetball",
    "foam brick": "061_foam_brick",
    "foam_brick": "061_foam_brick",
    "rubiks cube": "077_rubiks_cube",
    "rubiks_cube": "077_rubiks_cube",
}


def run_fp() -> np.ndarray:
    """Run the maintained package entry point and parse its 4x4 result."""
    process = subprocess.run(
        [sys.executable, "-m", "my_course_pkg.perception.foundationpose"],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
    )
    if process.returncode:
        raise RuntimeError(
            "FoundationPose failed: "
            f"rc={process.returncode}\n{process.stdout}\n{process.stderr}"
        )
    match = re.search(
        r"Estimated pose \(4x4 matrix\):\s*\n([\s\S]+?)\n"
        r"(?:$|Pose estimation failed)",
        process.stdout,
    )
    if not match:
        raise RuntimeError(f"Cannot parse pose matrix.\n{process.stdout}")
    rows = [
        line.strip().lstrip("[").rstrip("]").strip()
        for line in match.group(1).strip().splitlines()
        if line.strip()
    ]
    return np.array([[float(x) for x in row.split()] for row in rows[:4]])


def _decode_mask(annotation):
    segmentation = annotation["segmentation"]
    counts = segmentation["counts"]
    if isinstance(counts, str):
        counts = counts.encode()
    return mask_utils.decode(
        {"counts": counts, "size": segmentation["size"]}
    ).astype(bool)


def load_data(selected_json: Path, response_json: Path):
    target = (
        json.loads(selected_json.read_text(encoding="utf-8"))["candidates"][0]
        .strip()
        .lower()
        .replace("_", " ")
    )
    annotations = [
        annotation
        for annotation in json.loads(
            response_json.read_text(encoding="utf-8")
        )["annotations"]
        if annotation["class_name"].strip().lower().replace("_", " ") == target
    ]
    if not annotations:
        raise RuntimeError(f"No annotation for: {target}")
    annotation = annotations[0]
    return target, _decode_mask(annotation), annotation


def mesh_dir(target: str, mesh_root: Path) -> Path:
    mesh_name = NAME2MESH.get(target, target.replace(" ", "_"))
    return mesh_root / mesh_name


def project(pose, mesh_path, intrinsic, width, height):
    vertices = []
    with mesh_path.open(encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            if line.startswith("v "):
                values = line.split()
                vertices.append([float(values[1]), float(values[2]), float(values[3])])
    vertices = np.asarray(vertices)
    camera = (pose[:3, :3] @ vertices.T + pose[:3, 3:4]).T
    camera = camera[camera[:, 2] > 1e-3]
    uv = (intrinsic @ camera.T).T
    uv = uv[:, :2] / uv[:, 2:3]
    rounded = np.clip(
        np.round(uv).astype(int),
        (0, 0),
        (width - 1, height - 1),
    )
    return np.unique(rounded, axis=0), uv


def hull(points):
    points = sorted({(int(x), int(y)) for x, y in points})
    if len(points) <= 2:
        return points

    def cross(origin, first, second):
        return (first[0] - origin[0]) * (second[1] - origin[1]) - (
            first[1] - origin[1]
        ) * (second[0] - origin[0])

    lower = []
    for point in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def _parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pose-file", type=Path, help="saved 4x4 pose matrix")
    parser.add_argument("--selected-json", type=Path, default=DEFAULT_SELECTED_JSON)
    parser.add_argument("--response-json", type=Path, default=DEFAULT_RESPONSE_JSON)
    parser.add_argument("--rgb", type=Path, default=DEFAULT_RGB_PATH)
    parser.add_argument("--mesh-root", type=Path, default=DEFAULT_MESH_ROOT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OVERLAY_PATH)
    parser.add_argument("--fovy-deg", type=float, default=51.38)
    return parser


def main(args=None):
    args = _parser().parse_args(args)
    target, mask, annotation = load_data(args.selected_json, args.response_json)
    print(f"target={target}")
    pose = np.loadtxt(args.pose_file) if args.pose_file else run_fp()

    mesh_path = mesh_dir(target, args.mesh_root) / "textured.obj"
    if not mesh_path.exists():
        raise FileNotFoundError(str(mesh_path))
    image = Image.open(args.rgb).convert("RGB")
    width, height = image.size
    if mask.shape != (height, width):
        raise RuntimeError(
            "SAM2 mask/RGB size mismatch: "
            f"mask={mask.shape}, rgb={(height, width)}."
        )
    focal = (height / 2.0) / math.tan(math.radians(args.fovy_deg) / 2.0)
    intrinsic = np.array(
        [[focal, 0, width / 2], [0, focal, height / 2], [0, 0, 1]],
        dtype=np.float64,
    )
    unique_uv, all_uv = project(pose, mesh_path, intrinsic, width, height)
    hits = mask[unique_uv[:, 1], unique_uv[:, 0]].sum()
    ratio = hits / len(unique_uv) if len(unique_uv) else 0.0
    print(
        f"verts={len(all_uv)} unique={len(unique_uv)} hits={hits} "
        f"overlap={ratio:.3f}"
    )
    print(f"bbox={annotation['bbox']} mask_px={mask.sum()}")

    draw = ImageDraw.Draw(image, "RGBA")
    for x, y in all_uv[:: max(1, len(all_uv) // 4000)]:
        draw.ellipse((x - 1, y - 1, x + 1, y + 1), fill=(255, 0, 0, 140))
    projected_hull = hull(all_uv)
    if len(projected_hull) >= 3:
        draw.polygon(projected_hull, outline=(255, 0, 0, 255), fill=(255, 0, 0, 64))
    x0, y0, x1, y1 = annotation["bbox"]
    draw.rectangle((x0, y0, x1, y1), outline=(0, 255, 0, 255), width=3)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    image.save(args.output)
    print(f"overlay saved: {args.output}")
    print("PASS" if ratio >= 0.8 else "WARN" if ratio >= 0.5 else "FAIL")


if __name__ == "__main__":
    main()

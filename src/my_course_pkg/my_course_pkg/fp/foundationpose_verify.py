#!/usr/bin/env python3
"""Verify FoundationPose mesh projection against SAM2 mask."""
import argparse, json, math, re, subprocess, sys
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw
from pycocotools import mask as mask_utils
from my_course_pkg.ycb_models import resolve_ycb_visual_assets

W, H = 1280, 720
WS = Path("/home/ws")
FP_SCRIPT = WS / "src/my_course_pkg/my_course_pkg/fp/foundationpose.py"
SELECTED_JSON = WS / "outputs_local/vlm_sam2_test/selected_object.json"
RESPONSE_JSON = WS / "outputs_local/vlm_sam2_test/response.json"
RGB_PATH = WS / "outputs_local/rgbd_frame/rgb.png"
MESH_ROOT = WS / "src/my_course_pkg/YCB_Dataset/ycb"
OUT_PATH = WS / "src/my_course_pkg/my_course_pkg/fp/foundationpose_verify_out.png"
FY = (H / 2) / math.tan(math.radians(51.38) / 2)
K = np.array([[FY, 0, W / 2], [0, FY, H / 2], [0, 0, 1]], dtype=np.float64)


def run_fp() -> np.ndarray:
    p = subprocess.run([sys.executable, str(FP_SCRIPT)], capture_output=True, text=True)
    if p.returncode:
        raise RuntimeError(f"FP failed: rc={p.returncode}\n{p.stdout}\n{p.stderr}")
    m = re.search(
        r"Estimated pose \(4x4 matrix\):\s*\n([\s\S]+?)\n(?:$|Pose estimation failed)",
        p.stdout,
    )
    if not m:
        raise RuntimeError(f"Cannot parse pose matrix.\n{p.stdout}")
    rows = [line.strip().lstrip("[").rstrip("]").strip() for line in m.group(1).strip().splitlines() if line.strip()]
    return np.array([[float(x) for x in row.split()] for row in rows[:4]])


def load_data():
    target = (
        json.loads(SELECTED_JSON.read_text())["candidates"][0]
        .strip()
        .lower()
        .replace("_", " ")
    )
    anns = [
        a
        for a in json.loads(RESPONSE_JSON.read_text())["annotations"]
        if a["class_name"].strip().lower().replace("_", " ") == target
    ]
    if not anns:
        raise SystemExit(f"No annotation for: {target}")
    ann = anns[0]
    rle = {"counts": ann["segmentation"]["counts"].encode(), "size": ann["segmentation"]["size"]}
    mask = mask_utils.decode(rle).astype(bool)
    return target, mask, ann


def mesh_dir(target):
    return resolve_ycb_visual_assets(MESH_ROOT, target).directory


def project(pose, mesh_path):
    verts = []
    with open(mesh_path, encoding="utf-8", errors="ignore") as f:
        for line in f:
            if line.startswith("v "):
                v = line.split()
                verts.append([float(v[1]), float(v[2]), float(v[3])])
    verts = np.array(verts)
    cam = (pose[:3, :3] @ verts.T + pose[:3, 3:4]).T
    cam = cam[cam[:, 2] > 1e-3]
    uv = (K @ cam.T).T
    uv = uv[:, :2] / uv[:, 2:3]
    uv_round = np.clip(np.round(uv).astype(int), (0, 0), (W - 1, H - 1))
    return np.unique(uv_round, axis=0), uv


def hull(pts):
    pts = sorted({(int(x), int(y)) for x, y in pts})
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lo = []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    hi = []
    for p in reversed(pts):
        while len(hi) >= 2 and cross(hi[-2], hi[-1], p) <= 0:
            hi.pop()
        hi.append(p)
    return lo[:-1] + hi[:-1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose-file", type=Path, help="4x4 pose matrix file (skip FP run)")
    args = parser.parse_args()

    target, mask, ann = load_data()
    print(f"target={target}")

    pose = np.loadtxt(args.pose_file) if args.pose_file else run_fp()

    mesh_path = mesh_dir(target) / "textured.obj"

    uv_uq, uv_all = project(pose, mesh_path)
    hits = mask[uv_uq[:, 1], uv_uq[:, 0]].sum()
    ratio = hits / len(uv_uq) if len(uv_uq) else 0.0
    print(f"verts={len(uv_all)}  unique={len(uv_uq)}  hits={hits}  overlap={ratio:.3f}")
    print(f"bbox={ann['bbox']}  mask_px={mask.sum()}")

    img = Image.open(RGB_PATH).convert("RGB")
    draw = ImageDraw.Draw(img, "RGBA")
    for x, y in uv_all[:: max(1, len(uv_all) // 4000)]:
        draw.ellipse((x - 1, y - 1, x + 1, y + 1), fill=(255, 0, 0, 140))
    h = hull(uv_all)
    if len(h) >= 3:
        draw.polygon(h, outline=(255, 0, 0, 255), fill=(255, 0, 0, 64))
    x0, y0, x1, y1 = ann["bbox"]
    draw.rectangle((x0, y0, x1, y1), outline=(0, 255, 0, 255), width=3)
    img.save(OUT_PATH)
    print(f"overlay saved: {OUT_PATH}")

    if ratio >= 0.8:
        print("PASS")
    elif ratio >= 0.5:
        print("WARN")
    else:
        print("FAIL")


if __name__ == "__main__":
    main()

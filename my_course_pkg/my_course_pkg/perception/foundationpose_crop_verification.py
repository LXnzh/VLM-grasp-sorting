"""Optional crop-level VLM verification for FoundationPose masks."""

import json
from collections.abc import Callable
from pathlib import Path

import numpy as np
from PIL import Image

from my_course_pkg.env import env_bool as _shared_env_bool


def env_bool(name: str, default: bool = False) -> bool:
    """Compatibility wrapper for the package environment parser."""
    return _shared_env_bool(name, default)


def clamp_confidence(value) -> float:
    try:
        return max(0.0, min(1.0, float(value)))
    except (TypeError, ValueError):
        return 0.0


def mask_bbox_xyxy(mask: np.ndarray, padding_px: int = 12):
    ys, xs = np.where(mask)
    if len(xs) == 0:
        return None
    height, width = mask.shape
    x0 = max(0, int(xs.min()) - padding_px)
    y0 = max(0, int(ys.min()) - padding_px)
    x1 = min(width, int(xs.max()) + padding_px + 1)
    y1 = min(height, int(ys.max()) + padding_px + 1)
    if x1 <= x0 or y1 <= y0:
        return None
    return x0, y0, x1, y1


def save_candidate_crop(
    rgb_path: Path,
    output_dir: Path,
    mask: np.ndarray,
    candidate_index: int,
    *,
    padding_px: int,
    bbox_for_mask: Callable[[np.ndarray, int], tuple[int, int, int, int] | None] = (
        mask_bbox_xyxy
    ),
) -> Path | None:
    if not rgb_path.exists():
        return None
    bbox = bbox_for_mask(mask, padding_px)
    if bbox is None:
        return None
    x0, y0, x1, y1 = bbox
    image = Image.open(rgb_path).convert("RGB")
    if image.size != (mask.shape[1], mask.shape[0]):
        return None
    crop = image.crop((x0, y0, x1, y1))
    crop_dir = output_dir / "crop_verification"
    crop_dir.mkdir(parents=True, exist_ok=True)
    crop_path = crop_dir / f"candidate_{candidate_index:03d}.png"
    crop.save(crop_path)
    return crop_path


def save_selected_mask_visualizations(
    sam2_response_json: Path,
    rgb_path: Path,
    mask: np.ndarray,
) -> None:
    """Save the selected instance mask and RGB overlay beside SAM2 output."""
    output_dir = sam2_response_json.parent
    output_dir.mkdir(parents=True, exist_ok=True)

    mask = np.asarray(mask, dtype=bool)
    mask_image = Image.fromarray(mask.astype(np.uint8) * 255, mode="L")
    mask_path = output_dir / "selected_mask.png"
    mask_image.save(mask_path)

    if not rgb_path.exists():
        print(
            f"Selected mask saved: {mask_path}. "
            f"Overlay skipped because RGB image does not exist: {rgb_path}"
        )
        return

    rgb_image = Image.open(rgb_path).convert("RGB")
    if rgb_image.size != (mask.shape[1], mask.shape[0]):
        print(
            f"Selected mask saved: {mask_path}. Overlay skipped because RGB "
            f"size {rgb_image.size} does not match mask size "
            f"{(mask.shape[1], mask.shape[0])}."
        )
        return

    rgb = np.asarray(rgb_image, dtype=np.uint8).copy()
    highlight = np.array([0, 255, 0], dtype=np.float32)
    alpha = 0.55
    rgb[mask] = np.rint(
        (1.0 - alpha) * rgb[mask].astype(np.float32) + alpha * highlight
    ).astype(np.uint8)

    overlay_path = output_dir / "selected_mask_overlay.png"
    Image.fromarray(rgb, mode="RGB").save(overlay_path)
    print(f"Selected mask visualizations saved: {mask_path}, {overlay_path}")


def verify_crop_with_vlm(
    run_vlm_json: Callable[[Path, str], dict],
    crop_path: Path,
    target: str,
    selected_payload: dict,
    candidate_index: int,
    *,
    confidence_for_value: Callable[[object], float] = clamp_confidence,
) -> dict:
    target_region = selected_payload.get("target_region", {})
    visual_attributes = selected_payload.get("visual_attributes", {})
    prompt = f"""
You are verifying a single cropped object candidate for a robot pick task.

The robot target object is: {target!r}
Target region inferred from the full image:
{json.dumps(target_region, ensure_ascii=True)}
Target visual attributes inferred from the full image:
{json.dumps(visual_attributes, ensure_ascii=True)}

Look only at this cropped image. Decide whether the main object in the crop is
the target object the robot should pick.

Rules:
1. Return is_target=true only if the crop visually matches the target object.
2. Penalize confusing similar fruits if color/shape does not match.
3. If the crop appears to be a different object, return is_target=false.
4. confidence is the probability from 0.0 to 1.0 that this crop contains the target.
5. Output only valid JSON.
6. JSON format:
{{
  "is_target": true,
  "confidence": 0.0,
  "reason": "short reason"
}}
"""
    result = run_vlm_json(crop_path, prompt)
    is_target = bool(result.get("is_target", False))
    confidence = confidence_for_value(result.get("confidence", 0.0))
    return {
        "candidate_index": candidate_index,
        "is_target": is_target,
        "confidence": confidence if is_target else 0.0,
        "raw_confidence": confidence,
        "reason": str(result.get("reason", "")).strip(),
        "crop_path": str(crop_path),
        "raw": result,
    }


def apply_crop_vlm_verification(
    usable: list[dict],
    target: str,
    selected_payload: dict,
    *,
    verification_enabled: Callable[[], bool],
    api_key_is_present: Callable[[], bool],
    save_crop: Callable[[np.ndarray, int], Path | None],
    verify_crop: Callable[[Path, str, dict, int], dict],
) -> list[dict]:
    if not usable:
        return usable
    if not verification_enabled():
        return usable
    if not api_key_is_present():
        print("Skipping crop VLM verification: VLM_API_KEY/OPENAI_API_KEY is not set.")
        return usable

    try:
        for candidate_index, item in enumerate(usable, start=1):
            crop_path = save_crop(item["mask"], candidate_index)
            if crop_path is None:
                continue
            verification = verify_crop(
                crop_path,
                target,
                selected_payload,
                candidate_index,
            )
            item["crop_vlm"] = verification
            item["crop_vlm_score"] = verification["confidence"]
            print(
                "Crop VLM verification: "
                f"candidate={candidate_index}, "
                f"is_target={verification['is_target']}, "
                f"confidence={verification['raw_confidence']:.3f}, "
                f"target_score={verification['confidence']:.3f}, "
                f"reason={verification['reason']}"
            )
    except Exception as exc:
        print(f"Crop VLM verification failed; falling back to non-crop ranking: {exc}")
        for item in usable:
            item.pop("crop_vlm", None)
            item["crop_vlm_score"] = 0.0
    return usable

#!/usr/bin/env python3
"""FoundationPose pipeline wrapper.

Importing this module is side-effect free.  Call
FoundationPoseEstimationNode().run() to execute pose estimation.
"""

import base64
import io
import json
import math
import os
import time
import zipfile
from pathlib import Path

import numpy as np
from openai import APIStatusError, BadRequestError
from PIL import Image, ImageDraw, ImageFont
from pycocotools import mask as mask_utils
import requests

from my_course_pkg.paths import (
    DEPTH_PATH,
    FOUNDATIONPOSE_OUTPUT_DIR,
    RGB_PATH,
    SAM2_RESPONSE_JSON,
    SELECTED_OBJECT_JSON,
)
from my_course_pkg.perception.llm_sam2 import (
    VLM_MODEL,
    _get_vlm_client,
    _is_image_unsupported_error,
    _parse_json_object,
)
from my_course_pkg.perception.http import (
    short_response_text as _short_vlm_response_text,
)
from my_course_pkg.ycb_models import (
    NAME2MESH as NUMBERED_NAME2MESH,
    resolve_ycb_visual_assets,
)


W, H = 1280, 720
_fy = (H / 2) / math.tan(math.radians(51.38) / 2)
K = np.array(
    [
        [_fy, 0, W / 2],
        [0, _fy, H / 2],
        [0, 0, 1],
    ],
    dtype=np.float64,
)

SELECTED_OBJ_JSON = SELECTED_OBJECT_JSON
FP_OUTPUT_DIR = FOUNDATIONPOSE_OUTPUT_DIR
MESH_ROOT = Path("/home/ws/src/my_course_pkg/YCB_Dataset/ycb")
FP_URL = os.environ.get(
    "FOUNDATIONPOSE_URL",
    "http://172.22.222.220:5001",
)
RETRYABLE_STATUS_CODES = {429, 500, 502, 503, 504}
MASK_INDEX_ENV = "FOUNDATIONPOSE_MASK_INDEX"
AUTO_MASK_VERIFY_ENV = "FOUNDATIONPOSE_AUTO_MASK_VERIFY"
MASK_VERIFIER_SELECTED_BY = "vlm_candidate_verifier"
HIGHEST_SCORE_FALLBACK_SELECTED_BY = "highest_detection_score_fallback"
NAME2MESH = NUMBERED_NAME2MESH


SAM2_CLASS_ALIASES = {
    "tomato soup can": {"tomato soup"},
}


def _short_response_text(response: requests.Response) -> str:
    return " ".join(response.text.split())[:300]


def _post_with_retries(url: str, *, attempts: int = 3, **kwargs) -> requests.Response:
    last_error = None
    for attempt in range(1, attempts + 1):
        try:
            response = requests.post(url, **kwargs)
            if response.status_code not in RETRYABLE_STATUS_CODES:
                response.raise_for_status()
                return response
            last_error = RuntimeError(
                f"HTTP {response.status_code}: {_short_response_text(response)}"
            )
        except requests.RequestException as exc:
            last_error = exc

        if attempt < attempts:
            wait_sec = 2 ** (attempt - 1)
            print(f"Request to {url} failed ({last_error}); retrying in {wait_sec}s...")
            time.sleep(wait_sec)

    raise RuntimeError(
        f"Request to {url} failed after {attempts} attempts. Last error: {last_error}"
    ) from last_error


def _canonical_name(name: str) -> str:
    return name.strip().lower().replace("_", " ")


def _sam2_class_matches_target(
    class_name: str,
    target: str,
    accepted_sam2_class_names=None,
) -> bool:
    canonical_class = _canonical_name(class_name)
    canonical_target = _canonical_name(target)
    accepted_names = {canonical_target}
    accepted_names.update(SAM2_CLASS_ALIASES.get(canonical_target, set()))
    for accepted_name in accepted_sam2_class_names or ():
        if not isinstance(accepted_name, str) or not accepted_name.strip():
            continue
        accepted_names.add(_canonical_name(accepted_name))
    return canonical_class in accepted_names


def _decode_annotation_mask(annotation: dict) -> np.ndarray:
    segmentation = annotation["segmentation"]
    counts = segmentation["counts"]
    if isinstance(counts, str):
        counts = counts.encode()
    rle = {
        "counts": counts,
        "size": segmentation["size"],
    }
    return mask_utils.decode(rle).astype(bool)


def _optional_float(value):
    if value is None:
        return None
    return float(value)


def _optional_float_list(value):
    if value is None:
        return None
    return [float(item) for item in value]


def _env_enabled(name: str, default: bool = True) -> bool:
    raw_value = os.environ.get(name)
    if raw_value is None or raw_value.strip() == "":
        return default
    return raw_value.strip().lower() not in ("0", "false", "no", "off")


def _finite_candidate_score(candidate: dict, field: str) -> float | None:
    value = candidate["metadata"].get(field)
    try:
        score = float(value)
    except (TypeError, ValueError):
        return None
    return score if math.isfinite(score) else None


def _highest_detection_score_candidate(candidates: list[dict]) -> dict:
    if not candidates:
        raise ValueError("At least one mask candidate is required")

    for field in ("grounding_score", "score"):
        scored_candidates = []
        for candidate in candidates:
            score = _finite_candidate_score(candidate, field)
            if score is not None:
                index = int(candidate["metadata"]["index"])
                scored_candidates.append((score, -index, candidate))
        if scored_candidates:
            return max(scored_candidates, key=lambda item: (item[0], item[1]))[2]

    return min(candidates, key=lambda candidate: int(candidate["metadata"]["index"]))


def _image_data_url(path: Path) -> str:
    with path.open("rb") as handle:
        return "data:image/jpeg;base64," + base64.b64encode(handle.read()).decode(
            "ascii"
        )


def _card_resample_filter():
    if hasattr(Image, "Resampling"):
        return Image.Resampling.LANCZOS
    return Image.LANCZOS


def _candidate_number_font(size: int):
    for font_name in ("DejaVuSans-Bold.ttf", "Arial.ttf"):
        try:
            return ImageFont.truetype(font_name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def _target_visual_hint(
    target: str,
    visual_target_description: str | None = None,
) -> str:
    if visual_target_description:
        return (
            f"The target is {visual_target_description}. Prefer candidates that "
            "match this description and reject visually conflicting objects."
        )
    if target == "tomato soup can":
        return (
            "The target is a metal food can. In the robot's top-down view it may "
            "appear mainly as a gray or silver circular metal lid or bottom, and "
            "the printed side label may be hidden. Prefer the metal cylindrical "
            "can candidate over fruit-like round candidates."
        )
    return "Choose by the visible object appearance in the numbered crops."


class FoundationPoseEstimationNode:
    """Runs FoundationPose using the latest selected object and SAM2 response."""

    def __init__(
        self,
        selected_json: Path | str = SELECTED_OBJ_JSON,
        sam2_response_json: Path | str = SAM2_RESPONSE_JSON,
        rgb_path: Path | str = RGB_PATH,
        depth_path: Path | str = DEPTH_PATH,
        mesh_root: Path | str = MESH_ROOT,
        fp_url: str = FP_URL,
        output_dir: Path | str = FP_OUTPUT_DIR,
        mask_candidate_verifier=None,
    ):
        self.selected_json = Path(selected_json)
        self.sam2_response_json = Path(sam2_response_json)
        self.rgb_path = Path(rgb_path)
        self.depth_path = Path(depth_path)
        self.mesh_root = Path(mesh_root)
        self.fp_url = fp_url.rstrip("/")
        self.output_dir = Path(output_dir)
        self.mask_candidate_verifier = mask_candidate_verifier

    @property
    def pose_result_path(self) -> Path:
        return self.output_dir / "pose_result.json"

    @property
    def pose_error_path(self) -> Path:
        return self.output_dir / "pose_error.json"

    @property
    def mask_candidates_path(self) -> Path:
        return self.output_dir / "mask_candidates.json"

    @property
    def selected_mask_metadata_path(self) -> Path:
        return self.output_dir / "selected_mask_metadata.json"

    @property
    def mask_candidates_overlay_path(self) -> Path:
        return self.output_dir / "mask_candidates_overlay.jpg"

    @property
    def mask_verification_candidates_path(self) -> Path:
        return self.output_dir / "mask_verification_candidates.jpg"

    @property
    def mask_verification_result_path(self) -> Path:
        return self.output_dir / "mask_verification_result.json"

    def _load_target(self) -> str:
        selected = self._load_selected_object_metadata()
        target = selected.get("selected_object_name")
        if not target:
            target = selected["candidates"][0]
        return _canonical_name(target)

    def _load_selected_object_metadata(self) -> dict:
        selected = json.loads(self.selected_json.read_text(encoding="utf-8"))
        if not isinstance(selected, dict):
            raise RuntimeError("selected_object.json must contain a JSON object")
        return selected

    def _load_alias_mask_options(self):
        selected = self._load_selected_object_metadata()

        raw_alias = selected.get("matched_instruction_alias")
        if raw_alias is not None and not isinstance(raw_alias, str):
            raise RuntimeError(
                "matched_instruction_alias must be a string when present"
            )
        strict_alias = bool(raw_alias and raw_alias.strip())

        raw_accepted_names = selected.get("accepted_sam2_class_names", [])
        if not isinstance(raw_accepted_names, list) or any(
            not isinstance(name, str) or not name.strip()
            for name in raw_accepted_names
        ):
            raise RuntimeError(
                "accepted_sam2_class_names must be a list of non-empty strings"
            )
        accepted_names = tuple(
            _canonical_name(name) for name in raw_accepted_names
        )
        if strict_alias and not accepted_names:
            raise RuntimeError(
                "A matched instruction alias requires accepted_sam2_class_names"
            )

        visual_description = selected.get("visual_target_description")
        if visual_description is not None and (
            not isinstance(visual_description, str)
            or not visual_description.strip()
        ):
            raise RuntimeError(
                "visual_target_description must be a non-empty string when present"
            )

        return accepted_names, strict_alias, visual_description

    def _invalidate_previous_pose_result(self) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        for path in (
            self.pose_result_path,
            self.pose_error_path,
            self.selected_mask_metadata_path,
            self.mask_verification_candidates_path,
            self.mask_verification_result_path,
        ):
            try:
                path.unlink()
            except FileNotFoundError:
                pass

    def _candidate_metadata(
        self,
        *,
        filtered_index: int,
        source_annotation_index: int,
        annotation: dict,
        mask: np.ndarray,
    ) -> dict:
        grounding_score = _optional_float(annotation.get("grounding_score"))
        sam_score = _optional_float(annotation.get("score"))
        return {
            "index": filtered_index,
            "source_annotation_index": source_annotation_index,
            "class_name": annotation.get("class_name"),
            "score": grounding_score if grounding_score is not None else sam_score,
            "grounding_score": grounding_score,
            "sam_score": sam_score,
            "label": annotation.get("label"),
            "bbox": _optional_float_list(annotation.get("bbox")),
            "area_px": int(mask.sum()),
        }

    def _matching_mask_candidates(
        self,
        sam2: dict,
        target: str,
        accepted_sam2_class_names=(),
    ) -> list[dict]:
        candidates = []
        for source_index, annotation in enumerate(sam2["annotations"]):
            if not _sam2_class_matches_target(
                annotation["class_name"],
                target,
                accepted_sam2_class_names=accepted_sam2_class_names,
            ):
                continue

            mask = _decode_annotation_mask(annotation)
            candidates.append(
                {
                    "annotation": annotation,
                    "mask": mask,
                    "metadata": self._candidate_metadata(
                        filtered_index=len(candidates) + 1,
                        source_annotation_index=source_index,
                        annotation=annotation,
                        mask=mask,
                    ),
                }
            )
        return candidates

    def _write_mask_candidates(self, target: str, candidates: list[dict]) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        payload = {
            "target": target,
            "candidate_count": len(candidates),
            "override_env": MASK_INDEX_ENV,
            "candidates": [candidate["metadata"] for candidate in candidates],
        }
        self.mask_candidates_path.write_text(
            json.dumps(payload, indent=2),
            encoding="utf-8",
        )
        self._write_mask_candidates_overlay(candidates)

    def _write_mask_candidates_overlay(self, candidates: list[dict]) -> None:
        if not self.rgb_path.exists() or not candidates:
            return

        image = Image.open(self.rgb_path).convert("RGB")
        draw = ImageDraw.Draw(image)
        colors = [
            (255, 64, 64),
            (64, 192, 255),
            (255, 208, 64),
            (128, 255, 128),
            (255, 128, 255),
            (255, 160, 64),
        ]
        for candidate in candidates:
            metadata = candidate["metadata"]
            bbox = metadata.get("bbox")
            if not bbox:
                continue
            x1, y1, x2, y2 = [int(round(value)) for value in bbox]
            color = colors[(metadata["index"] - 1) % len(colors)]
            draw.rectangle([x1, y1, x2, y2], outline=color, width=4)
            text = str(metadata["index"])
            text_bg = [x1, y1, x1 + 18, y1 + 16]
            draw.rectangle(text_bg, fill=color)
            draw.text((x1, y1), text, fill=(0, 0, 0))

        image.save(self.mask_candidates_overlay_path)

    def _candidate_crop_box(self, candidate: dict, image_size: tuple[int, int]):
        width, height = image_size
        bbox = candidate["metadata"].get("bbox")
        if bbox:
            x1, y1, x2, y2 = bbox
        else:
            ys, xs = np.where(candidate["mask"])
            if xs.size == 0 or ys.size == 0:
                return None
            x1 = float(xs.min())
            y1 = float(ys.min())
            x2 = float(xs.max() + 1)
            y2 = float(ys.max() + 1)

        box_width = max(1.0, x2 - x1)
        box_height = max(1.0, y2 - y1)
        margin = max(8.0, 0.15 * max(box_width, box_height))
        left = max(0, int(math.floor(x1 - margin)))
        top = max(0, int(math.floor(y1 - margin)))
        right = min(width, int(math.ceil(x2 + margin)))
        bottom = min(height, int(math.ceil(y2 + margin)))
        if right <= left or bottom <= top:
            return None
        return left, top, right, bottom

    def _write_mask_verification_candidates(self, candidates: list[dict]) -> Path:
        if not self.rgb_path.exists():
            raise RuntimeError(
                f"RGB image for mask verification does not exist: {self.rgb_path}"
            )

        source = Image.open(self.rgb_path).convert("RGB")
        panel_width = 240
        panel_height = 270
        label_height = 42
        gap = 12
        cols = min(3, len(candidates))
        rows = int(math.ceil(len(candidates) / cols))
        card_width = cols * panel_width + (cols + 1) * gap
        card_height = rows * panel_height + (rows + 1) * gap
        crop_card = Image.new("RGB", (card_width, card_height), (245, 245, 245))
        font = _candidate_number_font(30)
        resample = _card_resample_filter()

        for item_index, candidate in enumerate(candidates):
            col = item_index % cols
            row = item_index // cols
            panel_x = gap + col * (panel_width + gap)
            panel_y = gap + row * (panel_height + gap)
            draw = ImageDraw.Draw(crop_card)
            draw.rectangle(
                [panel_x, panel_y, panel_x + panel_width, panel_y + panel_height],
                fill=(255, 255, 255),
                outline=(80, 80, 80),
                width=2,
            )
            number = str(candidate["metadata"]["index"])
            draw.rectangle(
                [panel_x, panel_y, panel_x + panel_width, panel_y + label_height],
                fill=(255, 220, 64),
            )
            draw.text(
                (panel_x + 12, panel_y + 5),
                f"Candidate {number}",
                fill=(0, 0, 0),
                font=font,
            )

            crop_box = self._candidate_crop_box(candidate, source.size)
            if crop_box is None:
                continue
            crop = source.crop(crop_box)
            crop.thumbnail(
                (panel_width - 20, panel_height - label_height - 20),
                resample,
            )
            paste_x = panel_x + (panel_width - crop.width) // 2
            paste_y = panel_y + label_height + (
                panel_height - label_height - crop.height
            ) // 2
            crop_card.paste(crop, (paste_x, paste_y))

        card = crop_card
        if self.mask_candidates_overlay_path.exists():
            scene = Image.open(self.mask_candidates_overlay_path).convert("RGB")
            scene.thumbnail((card_width, 420), resample)
            combined_width = max(card_width, scene.width)
            combined_height = scene.height + gap + crop_card.height
            card = Image.new("RGB", (combined_width, combined_height), (245, 245, 245))
            card.paste(scene, ((combined_width - scene.width) // 2, 0))
            card.paste(crop_card, ((combined_width - crop_card.width) // 2, scene.height + gap))

        self.output_dir.mkdir(parents=True, exist_ok=True)
        card.save(self.mask_verification_candidates_path)
        return self.mask_verification_candidates_path

    def _call_default_mask_candidate_verifier(
        self,
        target: str,
        candidates: list[dict],
        card_path: Path,
        visual_target_description: str | None = None,
    ) -> dict:
        candidate_indices = [
            str(candidate["metadata"]["index"]) for candidate in candidates
        ]
        prompt = f"""
You are helping a robot choose the correct object mask.

The image contains numbered candidate crops produced by the perception system.
Target object: {target}
Candidate numbers: {", ".join(candidate_indices)}

Choose the candidate that is most likely the target object.

Target-specific visual hint:
{_target_visual_hint(target, visual_target_description)}

Rules:
1. Return only valid JSON.
2. Use both the full-scene numbered overlay and the numbered candidate crops.
3. If exactly one candidate is most likely the target, return:
   {{"selected_index": <candidate number>, "confidence": "high"}}
4. If none of the candidates is the target, or if two candidates are equally
   plausible, return:
   {{"selected_index": null, "confidence": "low"}}
5. Do not explain your reasoning.
"""
        image_data_url = _image_data_url(card_path)
        vlm_client = _get_vlm_client()
        last_error = None
        attempts = 3
        for attempt in range(1, attempts + 1):
            try:
                response = vlm_client.chat.completions.create(
                    model=VLM_MODEL,
                    messages=[
                        {
                            "role": "user",
                            "content": [
                                {"type": "text", "text": prompt},
                                {
                                    "type": "image_url",
                                    "image_url": {"url": image_data_url},
                                },
                            ],
                        }
                    ],
                    temperature=0,
                )
                raw_response = response.choices[0].message.content
                parsed = _parse_json_object(raw_response)
                return {
                    "raw_response": raw_response,
                    "parsed": parsed,
                    "selected_index": parsed.get("selected_index"),
                    "confidence": parsed.get("confidence"),
                }
            except BadRequestError as exc:
                if _is_image_unsupported_error(exc):
                    raise RuntimeError(
                        f"The configured VLM model/API does not support image "
                        f"input for mask verification: model={VLM_MODEL}."
                    ) from exc
                raise
            except APIStatusError as exc:
                last_error = exc
                if exc.status_code not in RETRYABLE_STATUS_CODES or attempt == attempts:
                    raise RuntimeError(
                        "VLM mask verification request failed. "
                        f"HTTP {exc.status_code}: "
                        f"{_short_vlm_response_text(exc.response)}"
                    ) from exc
                wait_sec = 2 ** (attempt - 1)
                print(
                    f"VLM mask verification returned HTTP {exc.status_code}; "
                    f"retrying in {wait_sec}s..."
                )
                time.sleep(wait_sec)

        raise RuntimeError(f"VLM mask verification failed: {last_error}")

    def _call_mask_candidate_verifier(
        self,
        target: str,
        candidates: list[dict],
        card_path: Path,
        visual_target_description: str | None = None,
    ) -> dict:
        if self.mask_candidate_verifier is None:
            return self._call_default_mask_candidate_verifier(
                target,
                candidates,
                card_path,
                visual_target_description,
            )
        return self.mask_candidate_verifier(target, candidates, card_path)

    def _verification_decision(
        self,
        verification: dict,
        candidates: list[dict],
    ):
        selected_index = verification.get("selected_index")
        confidence = verification.get("confidence")
        if isinstance(confidence, str):
            confidence = confidence.strip().lower()

        if confidence != "high" or type(selected_index) is not int:
            return None, "uncertain", confidence

        for candidate in candidates:
            if candidate["metadata"]["index"] == selected_index:
                return candidate, "selected", confidence

        return None, "error", confidence

    def _write_mask_verification_result(
        self,
        *,
        target: str,
        candidates: list[dict],
        card_path: Path | None,
        verification: dict | None,
        decision: str,
        selected_candidate: dict | None,
        error: str | None = None,
    ) -> None:
        verification = verification or {}
        payload = {
            "target": target,
            "candidate_count": len(candidates),
            "candidate_card_path": str(card_path) if card_path else None,
            "raw_response": verification.get("raw_response"),
            "parsed": verification.get("parsed"),
            "selected_index": verification.get("selected_index"),
            "confidence": verification.get("confidence"),
            "decision": decision,
            "error": error,
            "selected_candidate": selected_candidate["metadata"]
            if selected_candidate is not None
            else None,
        }
        self.mask_verification_result_path.write_text(
            json.dumps(payload, indent=2),
            encoding="utf-8",
        )

    def _selected_candidate_from_verifier(
        self,
        target: str,
        candidates: list[dict],
        visual_target_description: str | None = None,
    ):
        if not _env_enabled(AUTO_MASK_VERIFY_ENV, default=True):
            return None

        card_path = None
        verification = None
        try:
            card_path = self._write_mask_verification_candidates(candidates)
            verification = self._call_mask_candidate_verifier(
                target,
                candidates,
                card_path,
                visual_target_description,
            )
            selected_candidate, decision, _confidence = self._verification_decision(
                verification,
                candidates,
            )
            self._write_mask_verification_result(
                target=target,
                candidates=candidates,
                card_path=card_path,
                verification=verification,
                decision=decision,
                selected_candidate=selected_candidate,
            )
            return selected_candidate
        except Exception as exc:
            self._write_mask_verification_result(
                target=target,
                candidates=candidates,
                card_path=card_path,
                verification=verification,
                decision="error",
                selected_candidate=None,
                error=str(exc),
            )
            print(f"Mask candidate verifier failed closed: {exc}")
            return None

    def _selected_candidate_from_override(self, candidates: list[dict]):
        raw_index = os.environ.get(MASK_INDEX_ENV)
        if raw_index is None or raw_index.strip() == "":
            return None

        try:
            selected_index = int(raw_index)
        except ValueError as exc:
            raise RuntimeError(
                f"{MASK_INDEX_ENV} must be an integer candidate index. "
                f"Got: {raw_index!r}. See {self.mask_candidates_path}."
            ) from exc

        for candidate in candidates:
            if candidate["metadata"]["index"] == selected_index:
                return candidate

        raise RuntimeError(
            f"{MASK_INDEX_ENV}={selected_index} does not match any candidate for "
            f"the selected target. Candidate metadata: {self.mask_candidates_path}"
        )

    def _write_selected_mask_metadata(
        self,
        target: str,
        candidate: dict,
        *,
        selected_by: str | None = None,
    ) -> None:
        if selected_by is None:
            selected_by = MASK_INDEX_ENV if os.environ.get(MASK_INDEX_ENV) else "single_match"
        payload = {
            "target": target,
            "selected_by": selected_by,
            "candidate": candidate["metadata"],
        }
        self.selected_mask_metadata_path.write_text(
            json.dumps(payload, indent=2),
            encoding="utf-8",
        )

    def _load_mask(self, target: str) -> np.ndarray:
        (
            accepted_sam2_class_names,
            strict_alias,
            visual_target_description,
        ) = self._load_alias_mask_options()
        sam2 = json.loads(self.sam2_response_json.read_text(encoding="utf-8"))
        candidates = self._matching_mask_candidates(
            sam2,
            target,
            accepted_sam2_class_names=accepted_sam2_class_names,
        )
        self._write_mask_candidates(target, candidates)

        if not candidates:
            raise RuntimeError(
                f"SAM2 did not detect the target object: {target}. "
                f"Candidate metadata: {self.mask_candidates_path}"
            )

        override_candidate = self._selected_candidate_from_override(candidates)
        if override_candidate is not None:
            self._write_selected_mask_metadata(target, override_candidate)
            return override_candidate["mask"]

        if len(candidates) > 1:
            if not _env_enabled(AUTO_MASK_VERIFY_ENV, default=True):
                raise RuntimeError(
                    f"SAM2 returned {len(candidates)} masks matching target "
                    f"{target!r}. Refusing to run FoundationPose or arm motion until "
                    "the mask is explicitly selected. Inspect "
                    f"{self.mask_candidates_path} and "
                    f"{self.mask_candidates_overlay_path}. Then rerun with "
                    f"{MASK_INDEX_ENV}=<candidate index>."
                )

            verifier_candidate = self._selected_candidate_from_verifier(
                target,
                candidates,
                visual_target_description,
            )
            if verifier_candidate is not None:
                self._write_selected_mask_metadata(
                    target,
                    verifier_candidate,
                    selected_by=MASK_VERIFIER_SELECTED_BY,
                )
                return verifier_candidate["mask"]

            if strict_alias:
                raise RuntimeError(
                    "Strict alias mask selection failed: the verifier did not "
                    "return a high-confidence unique candidate. Refusing the "
                    "highest detection-score fallback."
                )

            fallback_candidate = _highest_detection_score_candidate(candidates)
            self._write_selected_mask_metadata(
                target,
                fallback_candidate,
                selected_by=HIGHEST_SCORE_FALLBACK_SELECTED_BY,
            )
            print(
                "Mask candidate verifier did not make a usable selection; "
                "using highest detection-score candidate "
                f"{fallback_candidate['metadata']['index']}."
            )
            return fallback_candidate["mask"]

        selected = candidates[0]
        self._write_selected_mask_metadata(target, selected)
        return selected["mask"]

    def _mesh_dir(self, target: str) -> Path:
        return resolve_ycb_visual_assets(self.mesh_root, target).directory

    def _build_bundle(self, target: str, mask: np.ndarray) -> bytes:
        rgb = np.array(Image.open(self.rgb_path).convert("RGB")).astype(np.uint8)
        depth = np.load(self.depth_path).astype(np.float32)

        assets = resolve_ycb_visual_assets(self.mesh_root, target)

        buf = io.BytesIO()
        with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
            for name, arr in [
                ("rgb.npy", rgb),
                ("depth.npy", depth),
                ("mask.npy", mask),
                ("cam_K.npy", K),
            ]:
                arr_buf = io.BytesIO()
                np.save(arr_buf, arr)
                zf.writestr(name, arr_buf.getvalue())

            zf.write(assets.mesh, "mesh/textured.obj")
            zf.write(assets.material, "mesh/textured.mtl")
            zf.write(
                assets.texture,
                f"mesh/{assets.texture_archive_name}",
            )

        return buf.getvalue()

    def run(self) -> dict:
        target = self._load_target()
        print(f"Target object: {target}")

        self._invalidate_previous_pose_result()
        mask = self._load_mask(target)
        print(f"Mask pixel count: {mask.sum()}")

        bundle = self._build_bundle(target, mask)
        print(f"Sending request to {self.fp_url}/estimate_pose ...")

        response = _post_with_retries(
            f"{self.fp_url}/estimate_pose",
            files={"bundle": ("bundle.zip", bundle, "application/zip")},
            timeout=300,
        )
        result = response.json()

        self.output_dir.mkdir(parents=True, exist_ok=True)

        if result.get("success"):
            out_path = self.pose_result_path
            out_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
            pose = np.array(result["pose"])
            print("Pose estimation succeeded.")
            print("Estimated pose (4x4 matrix):")
            print(pose)
            print(f"Pose result saved: {out_path}")
        else:
            error_path = self.pose_error_path
            error_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
            print(f"Pose estimation failed: {result.get('error', response.text)[:200]}")
            print(f"Pose error saved: {error_path}")

        return result


def main() -> None:
    FoundationPoseEstimationNode().run()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import json
import os
from pathlib import Path

import numpy as np
from PIL import Image
import requests

from my_course_pkg.paths import (
    DEPTH_PATH,
    FOUNDATIONPOSE_OUTPUT_DIR,
    RGB_PATH,
    SAM2_RESPONSE_JSON,
    SELECTED_OBJECT_JSON,
)
from my_course_pkg.tasks.sorting.bin_locator import (
    SINGLE_BIN_MIN_PIXELS,
    _geometry_bin_mask,
    camera_intrinsic,
)

from my_course_pkg.perception.camera import camera_matrix
from my_course_pkg.perception.foundationpose_bundle import (
    NAME2MESH,
    build_bundle,
    mesh_dir,
)
from my_course_pkg.perception.foundationpose_crop_verification import (
    apply_crop_vlm_verification,
    clamp_confidence as _clamp_confidence_impl,
    env_bool as _env_bool_impl,
    mask_bbox_xyxy as _mask_bbox_xyxy_impl,
    save_candidate_crop,
    save_selected_mask_visualizations,
    verify_crop_with_vlm,
)
from my_course_pkg.perception.foundationpose_masks import (
    SAM2_CLASS_ALIASES,
    annotation_rank_score as _annotation_rank_score_impl,
    canonical_name as _canonical_name_impl,
    mask_centroid_xy as _mask_centroid_xy_impl,
    mask_in_pick_exclusion_region,
    relative_axis_selected_index as _relative_axis_selected_index_impl,
    sam2_class_matches_target as _sam2_class_matches_target_impl,
    sam2_detected_class_names as _sam2_detected_class_names_impl,
    select_relative_instance,
    target_region_is_user_specified as _target_region_is_user_specified_impl,
    validate_mask_geometry,
)
from my_course_pkg.perception.http import (
    RETRYABLE_STATUS_CODES as _DEFAULT_RETRYABLE_STATUS_CODES,
    post_with_retries,
    short_response_text,
)
from my_course_pkg.perception.sam_masks import decode_annotation_mask


SELECTED_OBJ_JSON = SELECTED_OBJECT_JSON
FP_OUTPUT_DIR = FOUNDATIONPOSE_OUTPUT_DIR
MESH_ROOT = Path("/home/ws/src/my_course_pkg/YCB_Dataset/ycb")
FP_URL = "http://172.22.222.220:5001"
RETRYABLE_STATUS_CODES = set(_DEFAULT_RETRYABLE_STATUS_CODES)


def _short_response_text(response: requests.Response) -> str:
    """Compatibility wrapper for FoundationPose HTTP diagnostics."""
    return short_response_text(response)


def _post_with_retries(url: str, *, attempts: int = 3, **kwargs) -> requests.Response:
    """Compatibility wrapper retaining FoundationPose's retry contract."""
    return post_with_retries(
        url,
        attempts=attempts,
        response_text=_short_response_text,
        retryable_status_codes=RETRYABLE_STATUS_CODES,
        **kwargs,
    )


def _canonical_name(name: str) -> str:
    """Compatibility wrapper for SAM2 target-name normalization."""
    return _canonical_name_impl(name)


def _sam2_class_matches_target(class_name: str, target: str) -> bool:
    """Compatibility wrapper for SAM2 target-class matching."""
    return _sam2_class_matches_target_impl(
        class_name,
        target,
        aliases=SAM2_CLASS_ALIASES,
    )


def _sam2_detected_class_names(sam2: dict) -> list[str]:
    """Compatibility wrapper for SAM2 response diagnostics."""
    return _sam2_detected_class_names_impl(sam2)


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
        mask_border_margin_px: int | None = None,
        min_mask_area_px: int | None = None,
    ):
        self.selected_json = Path(selected_json)
        self.sam2_response_json = Path(sam2_response_json)
        self.rgb_path = Path(rgb_path)
        self.depth_path = Path(depth_path)
        self.mesh_root = Path(mesh_root)
        self.fp_url = fp_url.rstrip("/")
        self.output_dir = Path(output_dir)
        self.mask_border_margin_px = (
            int(os.environ.get("FP_MASK_BORDER_MARGIN_PX", "5"))
            if mask_border_margin_px is None
            else int(mask_border_margin_px)
        )
        self.min_mask_area_px = (
            int(os.environ.get("FP_MASK_MIN_AREA_PX", "200"))
            if min_mask_area_px is None
            else int(min_mask_area_px)
        )

    def _load_target(self) -> str:
        selected = json.loads(self.selected_json.read_text(encoding="utf-8"))
        target = selected.get("selected_object_name")
        if not target:
            target = selected["candidates"][0]
        return _canonical_name(target)

    def _load_selected_payload(self) -> dict:
        return json.loads(self.selected_json.read_text(encoding="utf-8"))

    def _decode_annotation_mask(self, ann: dict) -> np.ndarray:
        return decode_annotation_mask(ann)

    def _validate_mask_geometry(self, mask: np.ndarray, target: str) -> None:
        validate_mask_geometry(
            mask,
            target,
            min_mask_area_px=self.min_mask_area_px,
            mask_border_margin_px=self.mask_border_margin_px,
        )

    def _load_pick_exclusion_mask(self) -> np.ndarray | None:
        if not self.rgb_path.exists():
            return None

        image_rgb = np.array(Image.open(self.rgb_path).convert("RGB")).astype(
            np.uint8
        )
        if not self.depth_path.exists():
            return None
        depth = np.load(self.depth_path).astype(np.float32)
        if depth.shape != image_rgb.shape[:2]:
            return None
        intrinsic = camera_intrinsic(image_rgb.shape[1], image_rgb.shape[0])
        # Only relative height is needed for this image-space exclusion.
        camera_to_height = np.diag([1.0, 1.0, -1.0, 1.0])
        try:
            bin_mask, _support_z = _geometry_bin_mask(
                depth,
                intrinsic,
                camera_to_height,
                min_pixels=SINGLE_BIN_MIN_PIXELS,
            )
        except RuntimeError:
            return None

        ys, xs = np.where(bin_mask)
        if len(xs) == 0:
            return None
        padding = int(os.environ.get("FP_PICK_EXCLUSION_PADDING_PX", "6"))
        height, width = bin_mask.shape
        x0 = max(0, int(xs.min()) - padding)
        x1 = min(width, int(xs.max()) + padding + 1)
        y0 = max(0, int(ys.min()) - padding)
        y1 = min(height, int(ys.max()) + padding + 1)

        exclusion = np.zeros_like(bin_mask, dtype=bool)
        exclusion[y0:y1, x0:x1] = True
        print(
            "Pick exclusion region from depth-detected bin: "
            f"bbox_xyxy=({x0}, {y0}, {x1 - 1}, {y1 - 1}), "
            f"geometry_pixels={int(bin_mask.sum())}"
        )
        return exclusion

    @staticmethod
    def _mask_centroid_xy(mask: np.ndarray) -> tuple[int, int] | None:
        return _mask_centroid_xy_impl(mask)

    @classmethod
    def _mask_in_pick_exclusion_region(
        cls,
        mask: np.ndarray,
        exclusion_mask: np.ndarray | None,
    ) -> bool:
        return mask_in_pick_exclusion_region(
            mask,
            exclusion_mask,
            centroid_for_mask=cls._mask_centroid_xy,
        )

    @staticmethod
    def _annotation_rank_score(ann: dict) -> float:
        return _annotation_rank_score_impl(ann)

    @staticmethod
    def _relative_axis_selected_index(
        usable: list[dict],
        label: str,
        axis: str,
    ) -> int | None:
        return _relative_axis_selected_index_impl(usable, label, axis)

    @classmethod
    def _select_relative_instance(
        cls,
        usable: list[dict],
        target_region: dict | None,
    ) -> list[dict]:
        return select_relative_instance(
            usable,
            target_region,
            selected_index_for_axis=cls._relative_axis_selected_index,
        )

    @staticmethod
    def _target_region_is_user_specified(target_region: dict | None) -> bool:
        return _target_region_is_user_specified_impl(target_region)

    @staticmethod
    def _env_bool(name: str, default: bool = False) -> bool:
        return _env_bool_impl(name, default)

    @staticmethod
    def _clamp_confidence(value) -> float:
        return _clamp_confidence_impl(value)

    @staticmethod
    def _mask_bbox_xyxy(mask: np.ndarray, padding_px: int = 12):
        return _mask_bbox_xyxy_impl(mask, padding_px)

    def _save_candidate_crop(self, mask: np.ndarray, candidate_index: int) -> Path | None:
        return save_candidate_crop(
            self.rgb_path,
            self.output_dir,
            mask,
            candidate_index,
            padding_px=int(os.environ.get("FP_CROP_PADDING_PX", "18")),
            bbox_for_mask=self._mask_bbox_xyxy,
        )

    def _save_selected_mask_visualizations(self, mask: np.ndarray) -> None:
        save_selected_mask_visualizations(
            self.sam2_response_json,
            self.rgb_path,
            mask,
        )

    def _run_crop_vlm_json(self, crop_path: Path, prompt: str) -> dict:
        from my_course_pkg.perception.llm_sam2 import _run_vlm_json

        return _run_vlm_json(str(crop_path), prompt, "crop-target-verification")

    def _verify_crop_with_vlm(
        self,
        crop_path: Path,
        target: str,
        selected_payload: dict,
        candidate_index: int,
    ) -> dict:
        return verify_crop_with_vlm(
            self._run_crop_vlm_json,
            crop_path,
            target,
            selected_payload,
            candidate_index,
            confidence_for_value=self._clamp_confidence,
        )

    def _apply_crop_vlm_verification(
        self,
        usable: list[dict],
        target: str,
        selected_payload: dict,
    ) -> list[dict]:
        return apply_crop_vlm_verification(
            usable,
            target,
            selected_payload,
            verification_enabled=lambda: self._env_bool(
                "FP_ENABLE_CROP_VLM_VERIFICATION",
                True,
            ),
            api_key_is_present=lambda: bool(
                os.environ.get("VLM_API_KEY") or os.environ.get("OPENAI_API_KEY")
            ),
            save_crop=self._save_candidate_crop,
            verify_crop=self._verify_crop_with_vlm,
        )

    def _load_mask(self, target: str) -> np.ndarray:
        sam2 = json.loads(self.sam2_response_json.read_text(encoding="utf-8"))
        selected_payload = self._load_selected_payload()
        target_region = selected_payload.get("target_region")
        anns = [
            ann
            for ann in sam2["annotations"]
            if _sam2_class_matches_target(ann["class_name"], target)
        ]
        if not anns:
            detected = _sam2_detected_class_names(sam2)
            suffix = f" Detected classes: {detected}" if detected else ""
            raise RuntimeError(
                f"SAM2 did not detect the target object: {target}.{suffix}"
            )

        errors = []
        exclusion_mask = self._load_pick_exclusion_mask()
        usable = []
        strict_user_region = self._target_region_is_user_specified(target_region)
        excluded_by_pick_region = 0
        for ann in anns:
            mask = self._decode_annotation_mask(ann)
            centroid_xy = self._mask_centroid_xy(mask)
            try:
                self._validate_mask_geometry(mask, target)
            except RuntimeError as exc:
                errors.append(str(exc))
                continue
            if self._mask_in_pick_exclusion_region(mask, exclusion_mask):
                excluded_by_pick_region += 1
                reason = (
                    f"SAM2 mask for {target!r} is inside the cyan box pick-exclusion "
                    "region."
                )
                errors.append(reason)
                continue
            usable.append(
                {
                    "ann": ann,
                    "mask": mask,
                    "centroid_xy": centroid_xy,
                    "rank_score": self._annotation_rank_score(ann),
                    "crop_vlm_score": 0.0,
                    "area": int(mask.sum()),
                }
            )

        if usable:
            if strict_user_region:
                usable = self._select_relative_instance(usable, target_region)

            usable = self._apply_crop_vlm_verification(
                usable,
                target,
                selected_payload,
            )
            usable = sorted(
                usable,
                key=lambda item: (
                    -item.get("crop_vlm_score", 0.0),
                    -item["rank_score"],
                    -item["area"],
                ),
            )
            selected = usable[0]
            ann = selected["ann"]
            if excluded_by_pick_region:
                print(
                    f"Excluded {excluded_by_pick_region} {target!r} SAM2 mask(s) "
                    "inside the cyan box pick-exclusion region."
                )
            print(
                "Selected SAM2 mask: "
                f"class={ann.get('class_name')!r}, "
                f"grounding_score={float(ann.get('grounding_score', 0.0)):.3f}, "
                f"score={float(ann.get('score', 0.0)):.3f}, "
                f"crop_vlm_score={selected.get('crop_vlm_score', 0.0):.3f}, "
                f"area={selected['area']}px"
            )
            self._save_selected_mask_visualizations(selected["mask"])
            return selected["mask"]

        raise RuntimeError(
            "SAM2 detected the target, but no target mask was usable for "
            f"FoundationPose. Reasons: {' | '.join(errors)}"
        )

    def _mesh_dir(self, target: str) -> Path:
        return mesh_dir(self.mesh_root, target, name_to_mesh=NAME2MESH)

    def _build_bundle(self, target: str, mask: np.ndarray) -> bytes:
        return build_bundle(
            rgb_path=self.rgb_path,
            depth_path=self.depth_path,
            target=target,
            mask=mask,
            mesh_dir_for_target=self._mesh_dir,
            camera_matrix_for_frame=camera_matrix,
        )

    def run(self) -> dict:
        target = self._load_target()
        print(f"Target object: {target}")

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
        result["target_object"] = target

        self.output_dir.mkdir(parents=True, exist_ok=True)
        out_path = self.output_dir / "pose_result.json"
        out_path.write_text(json.dumps(result, indent=2), encoding="utf-8")

        if result.get("success"):
            pose = np.array(result["pose"])
            print("Pose estimation succeeded.")
            print("Estimated pose (4x4 matrix):")
            print(pose)
            print(f"Pose result saved: {out_path}")
        else:
            print(f"Pose estimation failed: {result.get('error', response.text)[:200]}")

        return result


def main() -> None:
    FoundationPoseEstimationNode().run()


if __name__ == "__main__":
    main()

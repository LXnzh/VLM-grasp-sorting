"""SAM2-assisted target reacquisition for FoundationPose tracking."""

import json
import os

import numpy as np
from pycocotools import mask as mask_utils

from my_course_pkg.perception.foundationpose import _sam2_class_matches_target
from my_course_pkg.perception.llm_sam2 import (
    build_sam2_text_prompt,
    run_sam2_api,
)
from my_course_pkg.perception.sam_masks import decode_annotation_mask

from ._compat import legacy_override
from .motion_gate import mask_iou


class TargetLostError(RuntimeError):
    """Raised when the originally selected physical instance is unavailable."""


def _mask_is_usable_away_from_image_border(mask, margin_px=5):
    """Return whether a non-empty 2-D mask is complete within the image frame.

    FoundationPose rejects masks that reach the image edge because the object
    geometry is then incomplete.  Apply the same check while SAM2 re-anchors a
    tracked object so an erroneous background mask cannot replace a valid
    tracker mask just before pose estimation.
    """
    candidate = np.asarray(mask, dtype=bool)
    if candidate.ndim != 2 or not candidate.any():
        return False

    ys, xs = np.where(candidate)
    height, width = candidate.shape
    margin = max(0, int(margin_px))
    return (
        int(xs.min()) >= margin
        and int(ys.min()) >= margin
        and int(xs.max()) < width - margin
        and int(ys.max()) < height - margin
    )


def select_recovery_mask(
    candidates,
    predicted_mask,
    *,
    predicted_iou=0.20,
    allow_low_iou_single_instance_fallback=False,
):
    """Recover the one physical target from SAM2 proposal masks.

    SAM2 may emit duplicate or partial proposals for one object.  In explicit
    single-instance mode they are not treated as separate instances, and SAM2
    confidence is intentionally not used for identity selection.  In generic
    or multi-instance mode, low-overlap proposals remain a safe-stop.
    """
    candidates = [np.asarray(mask, dtype=bool) for mask in candidates]
    if not candidates:
        raise TargetLostError("SAM2 found no target-class instance.")
    if len(candidates) == 1:
        return candidates[0], "single target-class candidate"

    overlaps = [mask_iou(mask, predicted_mask) for mask in candidates]
    best_index = int(np.argmax(overlaps))
    if overlaps[best_index] >= float(predicted_iou):
        return (
            candidates[best_index],
            f"predicted-mask IoU={overlaps[best_index]:.3f}",
        )

    if not allow_low_iou_single_instance_fallback:
        raise TargetLostError(
            "SAM2 candidates did not match the tracked physical instance: "
            f"best_iou={overlaps[best_index]:.3f}, "
            f"required={float(predicted_iou):.3f}."
        )

    # The target is a single physical instance.  Keep the closest proposal
    # even when a low-rate camera makes the predicted mask too stale to meet
    # the normal IoU threshold.
    return (
        candidates[best_index],
        "single-instance best-overlap fallback: "
        f"mask_iou={overlaps[best_index]:.3f}",
    )


class TargetRecoveryMixin:
    """Methods that reacquire a selected instance after tracker loss."""

    @staticmethod
    def _decode_mask(annotation):
        return decode_annotation_mask(
            annotation,
            decoder=legacy_override("mask_utils", mask_utils),
        )

    def _target_mask_candidates(self, response_path):
        response = json.loads(response_path.read_text(encoding="utf-8"))
        candidates = []
        class_matches_target = legacy_override(
            "_sam2_class_matches_target",
            _sam2_class_matches_target,
        )
        for annotation in response.get("annotations", []):
            if class_matches_target(
                annotation.get("class_name", ""),
                self.target_name,
            ):
                candidates.append(self._decode_mask(annotation))
        return candidates

    def _select_reanchored_mask(self, response_path, predicted_mask):
        candidates = [
            mask
            for mask in self._target_mask_candidates(response_path)
            if mask.shape == predicted_mask.shape
        ]
        if not candidates:
            raise TargetLostError(
                f"SAM2 could not reacquire {self.target_name!r}."
            )

        # A text-prompted SAM2 call may occasionally label a large image-edge
        # background region as the target.  Do not let it overwrite an
        # interior mask that the high-rate tracker has already kept stable.
        # The 5 px default intentionally matches FoundationPose's validation;
        # the environment variable keeps both checks configurable together.
        border_margin = int(os.environ.get("FP_MASK_BORDER_MARGIN_PX", "5"))
        mask_is_usable = legacy_override(
            "_mask_is_usable_away_from_image_border",
            _mask_is_usable_away_from_image_border,
        )
        interior_candidates = [
            mask
            for mask in candidates
            if mask_is_usable(mask, border_margin)
        ]
        if not interior_candidates:
            if mask_is_usable(predicted_mask, border_margin):
                self._status(
                    "INSTANCE_REANCHORED: rejected "
                    f"{len(candidates)} border-touching SAM2 candidate(s); "
                    "using stable tracked mask."
                )
                return predicted_mask.copy()
            raise TargetLostError(
                f"SAM2 reacquired {self.target_name!r} only with masks that "
                "touch the image border."
            )

        # This project runs a single physical target.  SAM2 can still return
        # duplicate/partial masks for that one object, especially after a
        # low-rate frame jump.  Use the exact same single-instance policy as
        # LOST recovery: geometry chooses one proposal without a hard minimum
        # IoU, instead of falsely safe-stopping.
        selector = legacy_override("select_recovery_mask", select_recovery_mask)
        selected, reason = selector(
            interior_candidates,
            predicted_mask,
            predicted_iou=float(
                self.get_parameter("reanchor_min_iou").value
            ),
            allow_low_iou_single_instance_fallback=bool(
                self.get_parameter("single_instance_reanchor").value
            ),
        )
        self._status(f"INSTANCE_REANCHORED: {reason}")
        return selected

    def _recover_lost_target(self):
        maximum = int(
            self.get_parameter("lost_recovery_attempts").value
        )
        if self.recovery_attempts >= maximum:
            raise TargetLostError(
                f"Target recovery failed after {maximum} attempts."
            )
        self.recovery_attempts += 1
        self._status(
            "TARGET_LOST_RECOVERY: reacquiring from the latest RGB-D frame "
            f"(attempt {self.recovery_attempts}/{maximum})"
        )
        predicted_mask = self.tracking.tracker.current_mask()
        predicted_mask = self.tracking._full_mask(
            predicted_mask,
            self.rgbd_node.get_latest_rgbd()[0].shape[:2],
        )
        rgb, depth, stamp = self.rgbd_node.get_latest_rgbd()
        if rgb is None:
            raise TargetLostError(
                "No RGB-D frame is available for target recovery."
            )
        recovery_dir = (
            self.session_dir
            / "lost_recovery"
            / f"{int(stamp * 1e9)}"
        )
        rgb_path, _depth_path = self._save_frame(
            recovery_dir,
            rgb,
            depth,
        )
        sam2_dir = recovery_dir / "sam2"
        run_sam2 = legacy_override("run_sam2_api", run_sam2_api)
        build_prompt = legacy_override(
            "build_sam2_text_prompt",
            build_sam2_text_prompt,
        )
        run_sam2(
            str(rgb_path),
            build_prompt([self.target_name]),
            output_dir=sam2_dir,
        )
        candidates = self._target_mask_candidates(
            sam2_dir / "response.json"
        )
        selector = legacy_override("select_recovery_mask", select_recovery_mask)
        try:
            recovered, reason = selector(
                candidates,
                predicted_mask,
                predicted_iou=float(
                    self.get_parameter("reanchor_min_iou").value
                ),
                allow_low_iou_single_instance_fallback=bool(
                    self.get_parameter("single_instance_reanchor").value
                ),
            )
        except TargetLostError as exc:
            if self.recovery_attempts >= maximum:
                raise
            self._status(f"TARGET_RECOVERY_RETRY: {exc}")
            return False
        self.tracking.activate(rgb, depth, recovered, stamp)
        self._status(f"TARGET_RECOVERED: {reason}")
        return True

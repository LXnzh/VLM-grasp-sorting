"""High-rate target motion tracking without producing a grasp pose.

The tracker propagates one selected instance mask with sparse optical flow.
RGB motion, mask overlap and masked depth are used only as a safety gate.
The grasp pose is deliberately left to FoundationPose.
"""

from dataclasses import dataclass
import threading

import cv2
import numpy as np


ARMED = "ARMED"
MOVING = "MOVING"
STABILIZING = "STABILIZING"
STABLE = "STABLE"
LOST = "LOST"


@dataclass(frozen=True)
class MotionConfig:
    move_pixel: float = 2.5
    move_depth_m: float = 0.008
    move_iou: float = 0.94
    stable_pixel: float = 0.8
    stable_depth_m: float = 0.003
    stable_iou: float = 0.98
    stable_duration_s: float = 1.0
    stable_frames: int = 8
    min_features: int = 8
    max_features: int = 180
    lost_depth_jump_m: float = 0.20


@dataclass(frozen=True)
class MotionSnapshot:
    state: str
    movement_version: int
    disturbance_seen: bool
    last_move_stamp: float | None
    last_stop_stamp: float | None
    frame_stamp: float
    pixel_motion: float = 0.0
    depth_motion_m: float = 0.0
    mask_iou: float = 1.0
    loss_reason: str | None = None


def mask_iou(first, second):
    first = np.asarray(first, dtype=bool)
    second = np.asarray(second, dtype=bool)
    union = int(np.logical_or(first, second).sum())
    if union == 0:
        return 0.0
    return float(np.logical_and(first, second).sum()) / float(union)


class MotionStateMachine:
    """Convert high-rate motion observations into event timestamps."""

    def __init__(self, config=None, require_disturbance=False):
        self.config = config or MotionConfig()
        self.require_disturbance = bool(require_disturbance)
        self.state = ARMED
        self.movement_version = 0
        self.disturbance_seen = not self.require_disturbance
        self.last_move_stamp = None
        self.last_stop_stamp = None
        self.stationary_since = None
        self.stationary_frames = 0

    def observe(self, stamp, *, moving, tracking_ok=True):
        stamp = float(stamp)
        if not tracking_ok:
            self.state = LOST
            self.stationary_since = None
            self.stationary_frames = 0
            return self.state

        if moving:
            if self.state != MOVING:
                self.movement_version += 1
            self.state = MOVING
            self.disturbance_seen = True
            self.last_move_stamp = stamp
            self.stationary_since = None
            self.stationary_frames = 0
            return self.state

        if not self.disturbance_seen:
            self.state = ARMED
            return self.state

        if self.stationary_since is None:
            self.stationary_since = stamp
            self.stationary_frames = 1
        else:
            self.stationary_frames += 1

        elapsed = max(0.0, stamp - self.stationary_since)
        if (
            elapsed >= self.config.stable_duration_s
            and self.stationary_frames >= self.config.stable_frames
        ):
            self.state = STABLE
            self.last_stop_stamp = stamp
        else:
            self.state = STABILIZING
        return self.state


class MaskMotionTracker:
    """Propagate a binary mask and feed a motion state machine."""

    def __init__(
        self,
        initial_gray,
        initial_depth,
        initial_mask,
        initial_stamp,
        camera_matrix,
        config=None,
        require_disturbance=False,
    ):
        self.config = config or MotionConfig()
        self.gate = MotionStateMachine(self.config, require_disturbance)
        self.gray = np.asarray(initial_gray, dtype=np.uint8)
        self.mask = np.asarray(initial_mask, dtype=bool)
        self.camera_matrix = np.asarray(camera_matrix, dtype=float).reshape(3, 3)
        self.frame_stamp = float(initial_stamp)
        self.depth_center = self._depth_center(initial_depth, self.mask)
        self.points = self._features(self.gray, self.mask)
        self._lock = threading.Lock()
        self._metrics = (0.0, 0.0, 1.0)
        self._tracking_lost = False
        self._loss_reason = None

        lost_depth_jump_m = float(self.config.lost_depth_jump_m)
        if (
            not np.isfinite(lost_depth_jump_m)
            or lost_depth_jump_m <= 0.0
        ):
            raise ValueError(
                "lost_depth_jump_m must be finite and positive."
            )

    def _features(self, gray, mask):
        points = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=self.config.max_features,
            qualityLevel=0.01,
            minDistance=4,
            mask=(mask.astype(np.uint8) * 255),
            blockSize=5,
        )
        if points is None:
            return np.empty((0, 1, 2), dtype=np.float32)
        return points.astype(np.float32)

    def _depth_center(self, depth, mask):
        if depth is None:
            return None
        depth = np.asarray(depth, dtype=float)
        valid = mask & np.isfinite(depth) & (depth > 0.05) & (depth < 5.0)
        ys, xs = np.where(valid)
        if len(xs) < 20:
            return None
        z = float(np.median(depth[valid]))
        u = float(np.median(xs))
        v = float(np.median(ys))
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
        return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])

    @staticmethod
    def _warp_mask(mask, affine):
        height, width = mask.shape
        warped = cv2.warpAffine(
            mask.astype(np.uint8),
            affine,
            (width, height),
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
        )
        return warped.astype(bool)

    def update(self, gray, depth, stamp):
        gray = np.asarray(gray, dtype=np.uint8)
        stamp = float(stamp)
        with self._lock:
            if stamp <= self.frame_stamp:
                return self.snapshot()
            if gray.shape != self.gray.shape:
                raise ValueError("Tracking frame size changed.")
            if self._tracking_lost:
                # Loss is deliberately latched until SAM2 supplies a new mask
                # and the owner replaces this tracker via activate(). Without
                # the latch, one following frame could silently change LOST
                # back to MOVING before the recovery loop observes it.
                self.frame_stamp = stamp
                return self.snapshot()

            if len(self.points) < self.config.min_features:
                self.points = self._features(self.gray, self.mask)
            if len(self.points) < self.config.min_features:
                self._mark_lost(
                    stamp,
                    "insufficient tracked features before optical flow",
                )
                return self.snapshot()

            current, status, _ = cv2.calcOpticalFlowPyrLK(
                self.gray,
                gray,
                self.points,
                None,
                winSize=(21, 21),
                maxLevel=3,
                criteria=(
                    cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                    30,
                    0.01,
                ),
            )
            if current is None or status is None:
                good_old = np.empty((0, 2), dtype=np.float32)
                good_new = np.empty((0, 2), dtype=np.float32)
            else:
                valid = status.reshape(-1).astype(bool)
                good_old = self.points.reshape(-1, 2)[valid]
                good_new = current.reshape(-1, 2)[valid]

            if len(good_new) < self.config.min_features:
                self._mark_lost(
                    stamp,
                    "insufficient valid optical-flow features",
                )
                return self.snapshot()

            affine, inliers = cv2.estimateAffinePartial2D(
                good_old,
                good_new,
                method=cv2.RANSAC,
                ransacReprojThreshold=3.0,
            )
            if affine is None:
                shift = np.median(good_new - good_old, axis=0)
                affine = np.array(
                    [[1.0, 0.0, shift[0]], [0.0, 1.0, shift[1]]],
                    dtype=float,
                )

            warped_mask = self._warp_mask(self.mask, affine)
            overlap = mask_iou(self.mask, warped_mask)
            displacements = np.linalg.norm(good_new - good_old, axis=1)
            pixel_motion = float(np.median(displacements))
            new_depth_center = self._depth_center(depth, warped_mask)
            if depth is not None and new_depth_center is None:
                self._metrics = (pixel_motion, 0.0, overlap)
                self._mark_lost(
                    stamp,
                    "tracked mask has insufficient valid depth",
                )
                return self.snapshot()
            depth_motion = (
                float(np.linalg.norm(new_depth_center - self.depth_center))
                if new_depth_center is not None and self.depth_center is not None
                else 0.0
            )
            self._metrics = (pixel_motion, depth_motion, overlap)

            if depth_motion > float(self.config.lost_depth_jump_m):
                self._mark_lost(
                    stamp,
                    "masked depth jump "
                    f"{depth_motion:.4f}m exceeds "
                    f"{float(self.config.lost_depth_jump_m):.4f}m",
                )
                return self.snapshot()

            moving = (
                pixel_motion >= self.config.move_pixel
                or depth_motion >= self.config.move_depth_m
                or overlap <= self.config.move_iou
            )
            quiet = (
                pixel_motion <= self.config.stable_pixel
                and depth_motion <= self.config.stable_depth_m
                and overlap >= self.config.stable_iou
            )
            # The hysteresis band is conservatively treated as motion.
            self.gate.observe(stamp, moving=(moving or not quiet))

            self.gray = gray
            self.mask = warped_mask
            self.frame_stamp = stamp
            if new_depth_center is not None:
                self.depth_center = new_depth_center
            self.points = self._features(gray, warped_mask)
            return self.snapshot()

    def _mark_lost(self, stamp, reason):
        self.frame_stamp = float(stamp)
        self._tracking_lost = True
        self._loss_reason = str(reason)
        self.gate.observe(stamp, moving=False, tracking_ok=False)

    def invalidate(self, stamp, reason):
        """Latch an externally detected tracking failure."""
        with self._lock:
            self._mark_lost(stamp, reason)
            return self.snapshot()

    def snapshot(self):
        pixel_motion, depth_motion, overlap = self._metrics
        return MotionSnapshot(
            state=self.gate.state,
            movement_version=self.gate.movement_version,
            disturbance_seen=self.gate.disturbance_seen,
            last_move_stamp=self.gate.last_move_stamp,
            last_stop_stamp=self.gate.last_stop_stamp,
            frame_stamp=self.frame_stamp,
            pixel_motion=pixel_motion,
            depth_motion_m=depth_motion,
            mask_iou=overlap,
            loss_reason=self._loss_reason,
        )

    def current_mask(self):
        with self._lock:
            return self.mask.copy()

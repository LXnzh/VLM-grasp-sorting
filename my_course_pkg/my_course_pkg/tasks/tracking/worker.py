"""High-rate RGB-D target tracking worker."""

from collections import deque
from dataclasses import dataclass
import threading

import cv2
import numpy as np
import rclpy

from ._compat import legacy_override
from .motion_gate import LOST, STABLE, MaskMotionTracker


class TargetMovedError(RuntimeError):
    """Raised when a motion event invalidates perception or execution."""


@dataclass(frozen=True)
class TrackedFrame:
    rgb: np.ndarray
    depth: np.ndarray
    mask: np.ndarray
    stamp: float
    movement_version: int
    last_stop_stamp: float


class HighRateTrackingWorker:
    """Poll synchronized RGB-D frames and maintain one physical instance."""

    def __init__(
        self,
        rgbd_node,
        *,
        config,
        scale=0.5,
        poll_rate=30.0,
        history_frames=600,
        require_disturbance=False,
    ):
        self.rgbd_node = rgbd_node
        self.config = config
        self.scale = float(scale)
        self.period = 1.0 / float(poll_rate)
        self.history = deque(maxlen=int(history_frames))
        self.require_disturbance = bool(require_disturbance)
        self.tracker = None
        self.initial_target_camera_point = None
        self.latest_tracked = None
        self.last_stamp = 0.0
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _small_gray(self, rgb):
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        return cv2.resize(
            gray,
            None,
            fx=self.scale,
            fy=self.scale,
            interpolation=cv2.INTER_AREA,
        )

    def _small_depth(self, depth):
        return cv2.resize(
            np.asarray(depth, dtype=np.float32),
            None,
            fx=self.scale,
            fy=self.scale,
            interpolation=cv2.INTER_NEAREST,
        )

    def _small_mask(self, mask):
        return cv2.resize(
            np.asarray(mask, dtype=np.uint8),
            None,
            fx=self.scale,
            fy=self.scale,
            interpolation=cv2.INTER_NEAREST,
        ).astype(bool)

    @staticmethod
    def _full_mask(mask, shape):
        return cv2.resize(
            np.asarray(mask, dtype=np.uint8),
            (shape[1], shape[0]),
            interpolation=cv2.INTER_NEAREST,
        ).astype(bool)

    def _scaled_camera_matrix(self):
        matrix = np.asarray(self.rgbd_node.K, dtype=float).copy()
        matrix[0, :] *= self.scale
        matrix[1, :] *= self.scale
        matrix[2, 2] = 1.0
        return matrix

    def _loop(self):
        while rclpy.ok() and not self.stop_event.is_set():
            rgb, depth, stamp = self.rgbd_node.get_latest_rgbd()
            if rgb is None or stamp <= self.last_stamp:
                self.stop_event.wait(self.period)
                continue
            gray = self._small_gray(rgb)
            small_depth = self._small_depth(depth)
            with self.lock:
                self.last_stamp = float(stamp)
                if self.tracker is None:
                    self.history.append((float(stamp), gray))
                else:
                    snapshot = self.tracker.update(gray, small_depth, stamp)
                    if snapshot.state == LOST:
                        # Keep frames arriving while SAM2 performs a recovery
                        # request, so a recovered mask can catch up afterward.
                        self.history.append((float(stamp), gray))
                    full_mask = self._full_mask(
                        self.tracker.current_mask(),
                        rgb.shape[:2],
                    )
                    self.latest_tracked = (
                        rgb.copy(),
                        depth.copy(),
                        full_mask,
                        float(stamp),
                        snapshot,
                    )
            self.stop_event.wait(self.period)

    def activate(self, rgb, depth, mask, stamp):
        gray = self._small_gray(rgb)
        small_depth = self._small_depth(depth)
        small_mask = self._small_mask(mask)
        tracker_type = legacy_override("MaskMotionTracker", MaskMotionTracker)
        with self.lock:
            self.tracker = tracker_type(
                gray,
                small_depth,
                small_mask,
                stamp,
                self._scaled_camera_matrix(),
                config=self.config,
                require_disturbance=self.require_disturbance,
            )
            self.initial_target_camera_point = (
                None
                if self.tracker.depth_center is None
                else self.tracker.depth_center.copy()
            )
            # VLM and SAM2 may take seconds. Replay every buffered camera frame
            # after the selection image before switching to live tracking.
            for buffered_stamp, buffered_gray in list(self.history):
                if buffered_stamp > stamp:
                    self.tracker.update(
                        buffered_gray,
                        None,
                        buffered_stamp,
                    )
            self.history.clear()
            self.latest_tracked = None

    def snapshot(self):
        with self.lock:
            if self.tracker is None:
                return None
            return self.tracker.snapshot()

    def stable_frame(self):
        with self.lock:
            if self.tracker is None or self.latest_tracked is None:
                return None
            rgb, depth, mask, stamp, snapshot = self.latest_tracked
            current = self.tracker.snapshot()
            if current.state != STABLE:
                return None
            if current.last_stop_stamp is None or stamp < current.last_stop_stamp:
                return None
            return TrackedFrame(
                rgb=rgb.copy(),
                depth=depth.copy(),
                mask=mask.copy(),
                stamp=stamp,
                movement_version=current.movement_version,
                last_stop_stamp=current.last_stop_stamp,
            )

    def assert_unchanged(self, movement_version, require_stable=True):
        """Validate a perception result against the current tracked target.

        During FoundationPose/VLM work, a transient optical-flow event can
        increment ``movement_version`` even though the target has already
        settled again by the time the result is ready.  For those perception
        checks, the current stable state is the authoritative signal.  During
        active robot motion the caller passes ``require_stable=False`` and we
        retain the stricter version check so a real movement still cancels the
        guarded action.
        """
        snapshot = self.snapshot()
        if snapshot is None:
            raise TargetMovedError("High-rate target tracker is not initialized.")
        if snapshot.state == LOST:
            reason = (
                f": {snapshot.loss_reason}"
                if snapshot.loss_reason
                else "."
            )
            raise TargetMovedError(
                f"Target instance tracking was lost{reason}"
            )
        if require_stable:
            if snapshot.state != STABLE:
                raise TargetMovedError(
                    f"Target is no longer stable: state={snapshot.state}."
                )
            # A prior transient event is tolerated once the current target is
            # stable again.  This keeps FoundationPose results from being
            # discarded solely because optical flow briefly jittered.
            return
        if snapshot.movement_version != movement_version:
            raise TargetMovedError(
                "A new target movement event invalidated the active result."
            )

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=2.0)

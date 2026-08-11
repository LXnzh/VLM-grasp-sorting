"""
RGB-D Perception Node — subscribe to Orbbec RGB + depth, cache the latest
synchronised frame for downstream perception (SAM → FoundationPose).

State:
  Step A (done):  RGB-D sync subscription + frame caching
  Step B (next):  SAM segmentation on the cached frame
  Step C (later): FoundationPose 6D pose estimation
"""

import math
import threading
import rclpy
import os
import cv2
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np

from my_course_pkg.paths import RGBD_FRAME_DIR


class RGBDPerceptionNode(Node):
    """RGB-D perception node — caches the latest aligned frame."""

    def __init__(self, node_name: str = 'rgbd_perception_node'):
        super().__init__(node_name)
        self.cbg = ReentrantCallbackGroup()
        self.bridge = CvBridge()

        self.rgb_sub = Subscriber(self, Image, '/camera_orbbec/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera_orbbec/depth/image_raw')
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=5,
            slop=1.0,
        )
        self.sync.registerCallback(self.rgbd_callback)

        self.lock = threading.Lock()
        self._latest_rgb = None       # np.ndarray (H, W, 3)
        self._latest_depth = None     # np.ndarray (H, W)
        self._latest_stamp = 0.0
        self._frame_count = 0

        self._init_camera_intrinsic()


    def _init_camera_intrinsic(self):
        width = 1280
        height = 720
        fovy_deg = 51.38
        fovy_rad = math.radians(fovy_deg)
        cx = width / 2.0
        cy = height / 2.0
        fy = (height / 2.0) / math.tan(fovy_rad / 2.0)
        fx = fy                     

        self.K = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0,  0,  1],
        ], dtype=np.float32)

        self.fx, self.fy = fx, fy
        self.cx, self.cy = cx, cy
        self.get_logger().info(
            f'Camera intrinsics: fx={fx:.1f} fy={fy:.1f} '
            f'cx={cx:.1f} cy={cy:.1f}'
        )


    def rgbd_callback(self, rgb_msg: Image, depth_msg: Image):
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        ts = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9

        with self.lock:
            self._latest_rgb = rgb.copy()
            self._latest_depth = depth.copy()
            self._latest_stamp = ts
            self._frame_count += 1

        if self._frame_count == 1:
            self.get_logger().info(
                f'First RGB-D frame received: {rgb.shape[1]}x{rgb.shape[0]}, '
                f'depth range=[{depth.min():.3f}, {depth.max():.3f}]m'
            )

        # Auto-save every 20 frames (~1 Hz) so the latest frame is always on disk
        if self._frame_count % 20 == 0:
            self.save_current_frame()



    def get_latest_rgbd(self):
        """Return a deep copy of (rgb, depth, timestamp), or (None,None,0.0)."""
        with self.lock:
            if self._latest_rgb is None:
                return None, None, 0.0
            return (
                self._latest_rgb.copy(),
                self._latest_depth.copy(),
                self._latest_stamp,
            )
    def save_current_frame(self, save_dir: str | os.PathLike | None = None):
        """
        Save the latest cached RGB-D frame.

        Outputs:
            rgb.png        - RGB image for visual inspection
            depth.npy      - raw depth array, float32, unit: meters
            depth_vis.png  - color-mapped depth image
            overlay.png    - RGB + depth visualization overlay
        """
        rgb, depth, ts = self.get_latest_rgbd()

        if rgb is None or depth is None:
            self.get_logger().error('No frame available to save.')
            return False

        if save_dir is None:
            save_dir = RGBD_FRAME_DIR
        save_dir = os.fspath(save_dir)
        os.makedirs(save_dir, exist_ok=True)
        # rgb is in RGB order, but OpenCV imwrite expects BGR.
        rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        cv2.imwrite(
            os.path.join(save_dir, 'rgb.png'),
            rgb_bgr
        )
        np.save(
            os.path.join(save_dir, 'depth.npy'),
            depth
        )

        # Prepare depth visualization.
        depth_vis = depth.copy()
        depth_vis[~np.isfinite(depth_vis)] = 0.0
        depth_vis[depth_vis < 0] = 0.0

        # Clip only for visualization.
        # Raw depth.npy is NOT clipped.
        max_depth = 2
        depth_clipped = np.clip(depth_vis, 0.0, max_depth)
        depth_norm = (depth_clipped / max_depth * 255.0).astype(np.uint8)

        # applyColorMap returns BGR image.
        depth_color_bgr = cv2.applyColorMap(
            depth_norm,
            cv2.COLORMAP_JET
        )

        cv2.imwrite(
            os.path.join(save_dir, 'depth_vis.png'),
            depth_color_bgr
        )

        # Overlay RGB and depth visualization.
        # Both must be BGR here.
        overlay_bgr = cv2.addWeighted(
            rgb_bgr,
            0.4,
            depth_color_bgr,
            0.6,
            0
        )

        cv2.imwrite(
            os.path.join(save_dir, 'overlay.png'),
            overlay_bgr
        )

        self.get_logger().info(
            f'Frame saved to {save_dir}/ '
            f'(stamp={ts:.3f}, rgb_shape={rgb.shape}, depth_shape={depth.shape})'
        )

        return True


# Backward-compatible name for pipeline-style imports.
RgbdFileSaveNode = RGBDPerceptionNode

def main(args=None):
    rclpy.init(args=args)
    node = RGBDPerceptionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""Move a MuJoCo object once or oscillate it continuously."""

import math
import threading
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import MultiThreadedExecutor

from sim_pick_place.sim_client_node import SimClientNode


def oscillation_offset(elapsed, period, amplitude_x, amplitude_y):
    """Return an XY sinusoidal offset for one continuous-motion sample."""
    phase = math.tau * float(elapsed) / float(period)
    scale = math.sin(phase)
    return float(amplitude_x) * scale, float(amplitude_y) * scale


class SimTargetNudgeNode(SimClientNode):
    def __init__(self):
        super().__init__("sim_target_nudge_node")
        self.declare_parameter("object_name", "tennis_ball")
        self.declare_parameter("delta_x", 0.05)
        self.declare_parameter("delta_y", 0.0)
        self.declare_parameter("duration", 2.0)
        self.declare_parameter("settle_time", 0.5)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("continuous", False)
        self.declare_parameter("amplitude_x", 0.05)
        self.declare_parameter("amplitude_y", 0.0)
        self.declare_parameter("period", 10.0)

        self.object_name = (
            str(self.get_parameter("object_name").value).strip().lower()
        )
        self.continuous = bool(self.get_parameter("continuous").value)
        self.duration = float(self.get_parameter("duration").value)
        self.settle_time = float(self.get_parameter("settle_time").value)
        self.amplitude_x = float(
            self.get_parameter("amplitude_x").value
        )
        self.amplitude_y = float(
            self.get_parameter("amplitude_y").value
        )
        self.oscillation_period = float(
            self.get_parameter("period").value
        )
        rate = float(self.get_parameter("publish_rate").value)
        if rate <= 0.0:
            raise ValueError("publish_rate must be positive.")
        if self.continuous:
            if self.oscillation_period <= 0.0:
                raise ValueError("period must be positive in continuous mode.")
            if self.amplitude_x == 0.0 and self.amplitude_y == 0.0:
                raise ValueError(
                    "At least one oscillation amplitude must be nonzero."
                )
        elif self.duration <= 0.0 or self.settle_time < 0.0:
            raise ValueError(
                "duration must be positive and settle_time nonnegative."
            )

        self.publisher = self.create_publisher(
            PoseStamped,
            "/sim/object_pose_cmd",
            10,
        )
        self.initial_info = None
        self.start_time = None
        self.done = threading.Event()
        self.timer = self.create_timer(1.0 / rate, self._tick)

    @staticmethod
    def _smoothstep(value):
        value = min(1.0, max(0.0, float(value)))
        return value * value * (3.0 - 2.0 * value)

    def _tick(self):
        if self.done.is_set():
            return
        if self.initial_info is None:
            info = self._get_objects_info().get(self.object_name)
            if info is None:
                return
            self.initial_info = dict(info)
            self.start_time = time.monotonic()
            if self.continuous:
                self.get_logger().info(
                    f"Oscillating {self.object_name!r} continuously: "
                    f"amplitude=({self.amplitude_x:.3f}, "
                    f"{self.amplitude_y:.3f})m, "
                    f"period={self.oscillation_period:.2f}s."
                )
            else:
                self.get_logger().info(
                    f"Nudging {self.object_name!r} once over "
                    f"{self.duration:.2f}s."
                )

        elapsed = time.monotonic() - self.start_time
        x0, y0, z0 = self.initial_info["position"]
        qx, qy, qz, qw = self.initial_info["orientation"]
        if self.continuous:
            offset_x, offset_y = oscillation_offset(
                elapsed,
                self.oscillation_period,
                self.amplitude_x,
                self.amplitude_y,
            )
        else:
            progress = self._smoothstep(elapsed / self.duration)
            offset_x = (
                float(self.get_parameter("delta_x").value) * progress
            )
            offset_y = (
                float(self.get_parameter("delta_y").value) * progress
            )

        message = PoseStamped()
        message.header.frame_id = self.object_name
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = float(x0 + offset_x)
        message.pose.position.y = float(y0 + offset_y)
        message.pose.position.z = float(z0)
        message.pose.orientation.x = float(qx)
        message.pose.orientation.y = float(qy)
        message.pose.orientation.z = float(qz)
        message.pose.orientation.w = float(qw)
        self.publisher.publish(message)

        if (
            not self.continuous
            and elapsed >= self.duration + self.settle_time
        ):
            self.get_logger().info(
                f"Nudge complete; {self.object_name!r} is stationary."
            )
            self.done.set()


def main(args=None):
    rclpy.init(args=args)
    node = SimTargetNudgeNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.done.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        executor.shutdown()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

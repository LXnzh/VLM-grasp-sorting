import time
import json
import threading
import numpy as np
import rclpy
from rclpy.node import Node
import tf_transformations
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_srvs.srv import Trigger
from std_msgs.msg import String
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from visualization_msgs.msg import MarkerArray
import tf2_ros
from tf2_ros import Buffer, TransformListener

from arm_api2_py.arm_api2_client import ArmApi2Client

from sim_pick_place.utils.helpers import Pose6D, _posestamped_msg_to_pose6d, _to_pose6d, _pose6d_to_posestamped_msg, _pose6d_to_transform_matrix, _transform_matrix_to_pose6d, _randomize_orientation_z
from sim_pick_place.utils.sim_reset_client import SimResetClient
from sim_pick_place.utils.plot_utils import plot_trajectory_with_frames
from sim_pick_place.utils.pick_place_utils import plan_pick_place_trajectory, generate_random_pose_here, interpolate_lin

class SimClientNode(Node):
    
    def __init__(self, node_name: str = 'sim_client_node'):
        super().__init__(node_name)

        self.objects_info = {}
        self.scene_desc_sub = self.create_subscription(
            MarkerArray,
            '/scene_description',
            self._scene_description_callback,
            10
        )
        
        # tf buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def close(self):
        pass

    def _get_transform(self, from_frame, to_frame):
        """
        Retrieves the transformation matrix between two coordinate frames using TF2.

        :param from_frame: The name of the source frame.
        :type from_frame: str
        :param to_frame: The name of the target frame.
        :type to_frame: str
        :return: A 4x4 numpy array representing the transformation from from_frame to to_frame. Returns the identity matrix if the transform lookup fails.
        :rtype: np.ndarray
        """

        try:
            transform = self.tf_buffer.lookup_transform(to_frame, from_frame, rclpy.time.Time())
            trans = transform.transform.translation
            rot = transform.transform.rotation

            # Convert to 4x4 Transformation Matrix
            T = np.eye(4)
            T[:3, 3] = [trans.x, trans.y, trans.z]
            quat = [rot.x, rot.y, rot.z, rot.w]
            T[:3, :3] = tf_transformations.quaternion_matrix(quat)[:3, :3]

            return T
        except tf2_ros.LookupException as e:
            return np.eye(4)  # Default to identity matrix if lookup fails

    def _scene_description_callback(self, msg):
        """Callback to process MarkerArray messages and update objects_info."""
        self.objects_info = {
            marker.text: {
                'name': marker.id,
                'position': (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z),
                'orientation': (marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w),
                'frame_id': marker.header.frame_id,
            }
            for marker in msg.markers
        }
    
    def _get_objects_info(self):
        """Return the latest dictionary of objects from the scene description."""
        return self.objects_info
    
    def _world_pose_for(self, name: str):
        # Get cube poses in world frame
        info = self._get_objects_info().get(name, None)
        if info is None:
            raise RuntimeError(f"Object '{name}' not found in scene description")
        pose_6d = _to_pose6d(info['position'], info['orientation'])
        T_world_frame = self._get_transform(info['frame_id'], 'world')
        T_world_obj = T_world_frame @ _pose6d_to_transform_matrix(pose_6d)
        return _transform_matrix_to_pose6d(T_world_obj) 



def main(args=None):
    rclpy.init(args=args)
    node = SimClientNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.close()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()

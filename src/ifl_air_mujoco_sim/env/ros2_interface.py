import cv2
import numpy as np
import threading
import time
import logging
import json
import struct
from collections.abc import Mapping
import mujoco

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import queue
from sensor_msgs.msg import JointState, Image, PointCloud2, PointField
from std_msgs.msg import Header, Float64MultiArray, String
from builtin_interfaces.msg import Time as RosTime
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, TransformStamped
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from env.mjcontrol_interface import MuJoCoInterface
from env.robotiq_adapter import RobotiqAdapter
from env.utils.pointcloud import rgbd_to_pcd


class UR10eRos2Interface:
    """ROS2 interface for UR10e simulation using rclpy.

    - Replaces previous roslibpy bridge with native rclpy Node.
    - Subscribes to '/forward_position_controller/commands' (std_msgs/Float64MultiArray)
    - Exposes action server '/scaled_joint_trajectory_controller/follow_joint_trajectory' (control_msgs/FollowJointTrajectory)
    - Publishes '/joint_states' as sensor_msgs/JointState
    """

    def __init__(self, sim_cfg, ros_host=None, ros_port=None):
        # initialize logger
        self.logger = logging.getLogger('UR10eRos2Interface')
        self.logger.setLevel(logging.INFO)

        # Simulation interface
        self.sim = MuJoCoInterface(
            model_path=sim_cfg['model_path'],
            camera_names=sim_cfg['camera_names'],
            headless=sim_cfg.get('headless', True),
            control_timestep=sim_cfg.get('control_timestep', 0.01),
            render_fps=sim_cfg.get('render_fps', 30),
            objects_config=sim_cfg.get('objects_config', []),
            random_object_count=sim_cfg.get('random_object_count', 0),
            fixed_object_names=sim_cfg.get('fixed_object_names', []),
            scene_mode=sim_cfg.get('scene_mode', 'mix'),
            scene_object_categories=sim_cfg.get('scene_object_categories', {}),
            assigned_object_names=sim_cfg.get('assigned_object_names', []),
            placement_slots=sim_cfg.get('placement_slots', []),
            classification_bins=sim_cfg.get('classification_bins'),
            camera_size=sim_cfg.get('camera_size', (1280, 720)),
        )
        self.cloud_arr = np.empty(
            307200,
            dtype=[
                ('x',   '<f4'),
                ('y',   '<f4'),
                ('z',   '<f4'),
                ('rgb', '<u4'),
            ],
        )

        self.nu = self.sim.nu

        self.publisher_joint_order = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self._arm_dof = len(self.publisher_joint_order)  # number of arm joints (excluding gripper)
        self._joint_ids = list(range(self._arm_dof))
        self._gripper_joint_id = self._arm_dof  # assuming gripper is the next joint after arm joints

        self._lock = threading.Lock()
        self._arm_target_pos = np.copy(self.sim.data.ctrl)[:self._arm_dof]
        self._active_traj = None
        self._traj_start_time = None
        self._traj_idx = 0
        self._running = True
        
        self.camera_1_name = None
        self.camera_2_name = None
        self._publish_pc_cam1 = bool(sim_cfg.get('enable_pointcloud_camera1', False))
        self._publish_pc_cam2 = bool(sim_cfg.get('enable_pointcloud_camera2', False))
        self._publish_depth_cam1 = bool(sim_cfg.get('enable_depth_camera1', False))
        self._publish_depth_cam2 = bool(sim_cfg.get('enable_depth_camera2', False))
        self.tf_parent_frame = sim_cfg.get('tf_parent_frame', 'base_link')
        self._pc_camera_tfs = self._parse_pointcloud_transform(
            sim_cfg.get('pointcloud_transform', None)
        )
        self._pc_profile_enabled = bool(sim_cfg.get('pointcloud_profile', False))
        self._pc_profile_log_every = int(sim_cfg.get('pointcloud_profile_log_every', 50))
        self._pc_profile_acc = {'project': 0.0, 'filter': 0.0, 'serialize': 0.0, 'points': 0, 'count': 0}
        self._pc_stride = max(1, int(sim_cfg.get('pointcloud_stride', 1)))
        
        if len(sim_cfg['camera_names']) >= 1:
            self.camera_1_name = sim_cfg['camera_names'][0]
        if len(sim_cfg['camera_names']) >= 2:
            self.camera_2_name = sim_cfg['camera_names'][1]
        self.camera_1_pub = None
        self.camera_2_pub = None
        self.camera_1_pc_pub = None
        self.camera_2_pc_pub = None
        self.camera_1_depth_pub = None
        self.camera_2_depth_pub = None

        # initialize rclpy and node
        try:
            rclpy.init()
        except Exception:
            # already initialized in this process
            pass

        self.node = Node('mujoco_ur10e_interface')
        self.tf_broadcaster = TransformBroadcaster(self.node)

        # Publishers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.js_pub = self.node.create_publisher(JointState, '/joint_states', 10)
        self.scene_pub = self.node.create_publisher(MarkerArray, '/scene_description', qos)
        self.scene_clearance_pub = self.node.create_publisher(
            MarkerArray,
            '/scene_clearance_bounds',
            qos,
        )
        self._last_scene_clearance_error_log_time = float('-inf')
        self._scene_clearance_profile_samples = []
        if self.camera_1_name is not None:
            self.camera_1_pub = self.node.create_publisher(Image, f'/{self.camera_1_name}/color/image_raw', qos)
            if self._publish_pc_cam1:
                self.camera_1_pc_pub = self.node.create_publisher(PointCloud2, f'/{self.camera_1_name}/depth_registered/points', qos)
            else:
                self.camera_1_pc_pub = None
            if self._publish_depth_cam1:
                self.camera_1_depth_pub = self.node.create_publisher(Image, f'/{self.camera_1_name}/depth/image_raw', qos)
            else:
                self.camera_1_depth_pub = None
        if self.camera_2_name is not None:
            self.camera_2_pub = self.node.create_publisher(Image, f'/{self.camera_2_name}/color/image_raw', qos)
            if self._publish_pc_cam2:
                self.camera_2_pc_pub = self.node.create_publisher(PointCloud2, f'/{self.camera_2_name}/depth_registered/points', qos)
            else:
                self.camera_2_pc_pub = None
            if self._publish_depth_cam2:
                self.camera_2_depth_pub = self.node.create_publisher(Image, f'/{self.camera_2_name}/depth/image_raw', qos)
            else:
                self.camera_2_depth_pub = None
        else:
            self.camera_2_pc_pub = None

        # Bounded queue for images (latest only)
        self._img_queue = queue.Queue(maxsize=1)
        self._pcd_queue = queue.Queue(maxsize=1)

        # Start a dedicated publisher thread
        self._img_pub_running = True
        self._img_pub_thread = threading.Thread(target=self._image_publisher_loop, daemon=True)
        self._img_pub_thread.start()
        
        self._pcd_pub_running = True
        self._pcd_pub_thread = threading.Thread(target=self._pcd_publisher_loop, daemon=True)
        self._pcd_pub_thread.start()

        # Subscriber for forward position commands
        self.cmd_sub = self.node.create_subscription(
            Float64MultiArray, 
            '/forward_position_controller/commands', 
            self.cb_forward_pos, 
            10
        )
        
        # Action server for trajectory following
        self._traj_action_server = ActionServer(self.node, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory', self._execute_traj_goal)

        # Declare parameter for custom reset specification (JSON string)
        self.node.declare_parameter('reset_spec', '{}')
        
        # Service servers for simulation reset
        self._reset_service = self.node.create_service(Trigger, '/reset_sim', self._handle_reset_sim)
        self._reset_custom_service = self.node.create_service(Trigger, '/reset_sim_custom', self._handle_reset_sim_custom)
        
        self.logger.info("Reset services created: /reset_sim and /reset_sim_custom")

        # Publishing throttle: do not exceed this rate
        sim_dt = float(self.sim.ctr_timestep) if hasattr(self.sim, 'ctr_timestep') else float(sim_cfg.get('control_timestep', 0.01))
        cfg_pub_hz = sim_cfg.get('ros_publish_hz', None) if isinstance(sim_cfg, dict) else None
        default_hz = min(1.0 / max(sim_dt, 1e-6), 50.0)
        self.publish_rate = float(cfg_pub_hz) if cfg_pub_hz is not None else default_hz
        self.publish_interval = 1.0 / self.publish_rate
        self._last_publish = 0.0

        # Camera cadence handled in _sim_loop
        self._cam_fps = float(sim_cfg.get('render_fps', 10.0))
        self._cam_period = 1.0 / max(self._cam_fps, 1e-6)
        self._last_camera_publish = 0.0
        cfg_pc_fps = sim_cfg.get('pointcloud_fps', self._cam_fps)
        self._pc_fps = float(cfg_pc_fps) if cfg_pc_fps is not None else self._cam_fps
        self._pc_period = 1.0 / max(self._pc_fps, 1e-6)
        self._next_pc_enqueue_t = time.perf_counter()


        # Optionally choose a smaller camera resolution to reduce cost
        self._cam_size = sim_cfg.get('camera_size', (640, 480))  # e.g. (640, 480) if your interface supports it

        self.gripper_adapter = RobotiqAdapter()
        self.gripper_adapter.start()

        # start rclpy spin in a background thread so action servers and any timers work
        self._rclpy_thread = threading.Thread(target=self._rclpy_spin, daemon=True)
        self._rclpy_thread.start()

        # start simulation loop thread
        self._thread = threading.Thread(target=self._sim_loop, daemon=True)
        self._thread.start()
        
        self.bridge = CvBridge()
        self.logger.info(
            "Camera fps=%.1f, pointcloud fps=%.1f, pointcloud stride=%d",
            self._cam_fps,
            self._pc_fps,
            self._pc_stride,
        )


    def cb_forward_pos(self, msg):
        """Callback for Float64MultiArray messages from rclpy subscriber."""
        with self._lock:
            data = msg.data
            arr = np.array(data, dtype=float)
            if arr.shape[0] == self._arm_dof:
                self._arm_target_pos = arr.copy()
                self._active_traj = None

    def cb_trajectory(self, msg):
        """Callback for JointTrajectory messages from roslibpy (dict)."""
        with self._lock:
            points = msg.get('points', [])
            if len(points) == 0:
                return
            # store the raw message; parsing happens in sim loop
            self._active_traj = msg
            self._traj_start_time = time.perf_counter()
            self._traj_idx = 0

    # --- rclpy helpers / action callbacks ---
    def _rclpy_spin(self):
        try:
            rclpy.spin(self.node)
        except Exception as e:
            self.logger.exception('rclpy spin failed: %s', e)

    def _image_publisher_loop(self):
        """Runs on its own thread. Pulls frames from queue and publishes."""
        last_time = time.time()
        while self._img_pub_running:
            start_time = time.time()
            try:
                payload = self._img_queue.get(timeout=0.05)
            except queue.Empty:
                continue
            
            (sec, nsec, camera_1_img, camera_2_img, depth_1_img, depth_2_img) = payload
            # Build and publish messages (serialize here, not in sim thread)
            if camera_1_img is not None:
                msg = self._create_camera_img_msg(sec, nsec, self.camera_1_name, camera_1_img)
                try:
                    self.camera_1_pub.publish(msg)
                except Exception:
                    pass
            if depth_1_img is not None and self.camera_1_depth_pub is not None:
                # Convert depth to 32-bit float in meters
                depth_msg = self._create_camera_img_msg(sec, nsec, self.camera_1_name, depth_1_img, encoding='32FC1')
                try:
                    self.camera_1_depth_pub.publish(depth_msg)
                except Exception:
                    pass
            if camera_2_img is not None:
                msg = self._create_camera_img_msg(sec, nsec, self.camera_2_name, camera_2_img)
                try:
                    self.camera_2_pub.publish(msg)
                except Exception:
                    pass
            if depth_2_img is not None and self.camera_2_depth_pub is not None:
                # Convert depth to 32-bit float in meters
                depth_msg = self._create_camera_img_msg(sec, nsec, self.camera_2_name, depth_2_img, encoding='32FC1')
                try:
                    self.camera_2_depth_pub.publish(depth_msg)
                except Exception:
                    pass
            elapsed = time.time() - start_time
            #print(f"Published images in {elapsed*1000:.1f} ms")
            interval = time.time() - last_time
            available_data = [camera_1_img is not None, camera_2_img is not None, depth_1_img is not None, depth_2_img is not None]
            #print(f"Image publishing interval: {interval*1000:.1f} ms, data available: {available_data}")
            last_time = time.time()

    def _create_camera_img_msg(self, sec, nsec, camera_name, camera_img, encoding='bgr8'):
        if encoding == 'bgr8':
            camera_img = cv2.cvtColor(camera_img, cv2.COLOR_RGB2BGR)
        else:
            camera_img = camera_img.astype(np.float32)  # ensure depth is float32 for 32FC1 encoding
        msg = self.bridge.cv2_to_imgmsg(camera_img, encoding=encoding)
        msg.header = Header()
        msg.header.stamp = RosTime(sec=sec, nanosec=nsec)
        msg.header.frame_id = camera_name
        return msg

    def _pcd_publisher_loop(self):
        """Publish point clouds off the sim thread."""
        while self._pcd_pub_running:
            try:
                stamp_sec, stamp_nsec, cloud_entries = self._pcd_queue.get(timeout=0.05)
            except queue.Empty:
                continue
            for entry in cloud_entries:
                msg = self._build_pointcloud2(
                    stamp_sec,
                    stamp_nsec,
                    entry['camera_name'],
                    entry['color'],
                    entry['depth'],
                    entry['intrinsics'],
                )
                if msg is None:
                    continue
                pub = None
                if entry['camera_name'] == self.camera_1_name:
                    pub = self.camera_1_pc_pub
                elif entry['camera_name'] == self.camera_2_name:
                    pub = self.camera_2_pc_pub
                if pub is not None:
                    try:
                        pub.publish(msg)
                    except Exception:
                        pass

    def _enqueue_image_pair(self, cam_1_img, cam_2_img, depth_1=None, depth_2=None, stamp=None):
        """Called from the SIM THREAD - push into queue, do not serialize here."""
        now = time.time() if stamp is None else stamp
        sec = int(now); nsec = int((now % 1) * 1e9)

        # If your renderer reuses buffers, copy *cheaply* or use a small ring buffer.
        # To be safe, make shallow copies only when needed:
        cam1 = None if cam_1_img is None else (cam_1_img if cam_1_img.flags['C_CONTIGUOUS'] and cam_1_img.dtype==np.uint8 else np.ascontiguousarray(cam_1_img.astype(np.uint8, copy=False)))
        cam2 = None if cam_2_img is None else (cam_2_img if cam_2_img.flags['C_CONTIGUOUS'] and cam_2_img.dtype==np.uint8 else np.ascontiguousarray(cam_2_img.astype(np.uint8, copy=False)))
        
        # Handle depth images (already using pre-allocated buffers, no copy needed)
        depth1 = depth_1 if self._publish_depth_cam1 else None
        depth2 = depth_2 if self._publish_depth_cam2 else None

        try:
            self._img_queue.put_nowait((sec, nsec, cam1, cam2, depth1, depth2))
        except queue.Full:
            # Drop frame instead of blocking physics
            pass

    def _enqueue_pointclouds(self, stamp_sec, stamp_nsec, cam_1_img, depth_1, cam_2_img, depth_2):
        """Queue point cloud construction for enabled cameras."""
        if not (self._publish_pc_cam1 or self._publish_pc_cam2):
            return

        entries = []
        if self._publish_pc_cam1 and self.camera_1_name is not None and cam_1_img is not None and depth_1 is not None:
            intr = self.sim.get_camera_intrinsics(self.camera_1_name, self._cam_size[0], self._cam_size[1])
            extr = self.sim.get_camera_extrinsics(self.camera_1_name)
            if intr is not None and extr is not None:
                # Copy only for point cloud thread (buffers will be reused in sim loop)
                entries.append({
                    'camera_name': self.camera_1_name,
                    'color': cam_1_img.copy() if cam_1_img.flags['OWNDATA'] else np.array(cam_1_img, copy=True),
                    'depth': depth_1.copy() if depth_1.flags['OWNDATA'] else np.array(depth_1, copy=True),
                    'intrinsics': intr,
                    'extrinsics': extr
                })

        if self._publish_pc_cam2 and self.camera_2_name is not None and cam_2_img is not None and depth_2 is not None:
            intr = self.sim.get_camera_intrinsics(self.camera_2_name, self._cam_size[0], self._cam_size[1])
            extr = self.sim.get_camera_extrinsics(self.camera_2_name)
            if intr is not None and extr is not None:
                # Copy only for point cloud thread (buffers will be reused in sim loop)
                entries.append({
                    'camera_name': self.camera_2_name,
                    'color': cam_2_img.copy() if cam_2_img.flags['OWNDATA'] else np.array(cam_2_img, copy=True),
                    'depth': depth_2.copy() if depth_2.flags['OWNDATA'] else np.array(depth_2, copy=True),
                    'intrinsics': intr,
                    'extrinsics': extr
                })

        if entries:
            try:
                self._pcd_queue.put_nowait((stamp_sec, stamp_nsec, entries))
            except queue.Full:
                pass

    def _build_pointcloud2(self, stamp_sec, stamp_nsec, camera_name, color_img, depth_img, intrinsics):
        """Convert rgb + depth to PointCloud2."""
        if color_img is None or depth_img is None:
            return None
        if intrinsics is None:
            return None

        profiling = self._pc_profile_enabled
        if profiling:
            t0 = time.perf_counter()
        
        if self._pc_stride > 1:
            stride = self._pc_stride
            color_img = color_img[::stride, ::stride]
            depth_img = depth_img[::stride, ::stride]
            intrinsics = np.array(intrinsics, dtype=np.float32, copy=True)
            intrinsics[0, 0] /= stride
            intrinsics[1, 1] /= stride
            intrinsics[0, 2] /= stride
            intrinsics[1, 2] /= stride


        rgbd = np.concatenate(
            [color_img.astype(np.float32), depth_img[..., np.newaxis].astype(np.float32)],
            axis=-1
        )
        rgbd = np.expand_dims(rgbd, axis=0)
        points, colors = rgbd_to_pcd(rgbd, intrinsics, None)

        if profiling:
            t_project_end = time.perf_counter()

        pts = points.reshape(-1, 3)
        cols = colors.reshape(-1, colors.shape[-1])

        valid = np.isfinite(pts).all(axis=1) & (pts[:, 2] > 0.0)
        pts = pts[valid]
        cols = cols[valid]
        if pts.shape[0] == 0:
            return None

        if profiling:
            t_filter_end = time.perf_counter()
            
        msg = self._points_to_cloud_msg(pts, cols, camera_name, stamp_sec, stamp_nsec)

        if profiling:
            t_serial_end = time.perf_counter()
            self._record_pointcloud_profile(
                t_project_end - t0,
                t_filter_end - t_project_end,
                t_serial_end - t_filter_end,
                pts.shape[0],
                camera_name
            )

        return msg
    
    def _points_to_cloud_msg(self, pts, cols, camera_name, stamp_sec, stamp_nsec):
        """Convert 3D points and colors to ROS2 PointCloud2 message.
        
        Parameters
        ----------
        pts : np.ndarray
            Point coordinates with shape (N, 3) as float32/float64.
        cols : np.ndarray
            RGB colors with shape (N, 3) as uint8 in range [0, 255].
        camera_name : str
            Camera frame identifier for the point cloud.
        stamp_sec : int
            Timestamp seconds.
        stamp_nsec : int
            Timestamp nanoseconds.
        
        Returns
        -------
        PointCloud2
            ROS2 PointCloud2 message with packed RGB as FLOAT32 (PCL style).
        
        Notes
        -----
        RGB values are packed into a uint32 (0x00RRGGBB) then reinterpreted
        as float32 without value conversion, following PCL convention.
        """
        n_points = pts.shape[0]

        msg = PointCloud2()
        msg.header = Header()
        #msg.header.stamp = RosTime(sec=stamp_sec, nanosec=stamp_nsec)
        msg.header.frame_id = camera_name

        msg.height = 1
        msg.width = n_points
        msg.is_bigendian = False
        msg.is_dense = False

        # NOTE: rgb is now FLOAT32 (packed bits)
        msg.fields = [
            PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        msg.point_step = 16  # 4 fields * 4 bytes
        msg.row_step = msg.point_step * n_points

        # Ensure dtypes
        pts_f32 = pts.astype(np.float32, copy=False)
        cols_u8 = np.clip(cols, 0, 255).astype(np.uint8, copy=False)

        # Pack RGB into uint32: 0x00RRGGBB
        rgb_u32 = (
            (cols_u8[:, 0].astype(np.uint32) << 16) |
            (cols_u8[:, 1].astype(np.uint32) << 8)  |
            (cols_u8[:, 2].astype(np.uint32))
        )

        # Reinterpret bits as float32 (no value conversion!)
        rgb_f32 = rgb_u32.view(np.float32)

        # Make / reuse structured array with float32 rgb
        # (If you already have self.cloud_arr, ensure its dtype matches this.)
        if (not hasattr(self, "cloud_arr")) or (self.cloud_arr.shape[0] != n_points) or (self.cloud_arr.dtype != np.dtype([
            ("x", np.float32), ("y", np.float32), ("z", np.float32), ("rgb", np.float32)
        ])):
            self.cloud_arr = np.empty(n_points, dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32), ("rgb", np.float32)])

        self.cloud_arr['x'] = pts_f32[:, 0]
        self.cloud_arr['y'] = pts_f32[:, 1]
        self.cloud_arr['z'] = pts_f32[:, 2]
        self.cloud_arr['rgb'] = rgb_f32
        msg._data = self.cloud_arr.tobytes()        # direct access to protected member for performance
        return msg

    def _record_pointcloud_profile(self, project_t, filter_t, serialize_t, n_pts, camera_name):
        """Accumulate profiling timings and log periodically."""
        if not self._pc_profile_enabled:
            return
        acc = self._pc_profile_acc
        acc['project'] += project_t
        acc['filter'] += filter_t
        acc['serialize'] += serialize_t
        acc['points'] += n_pts
        acc['count'] += 1
        if acc['count'] >= max(1, self._pc_profile_log_every):
            avg_project = (acc['project'] / acc['count']) * 1000.0
            avg_filter = (acc['filter'] / acc['count']) * 1000.0
            avg_serialize = (acc['serialize'] / acc['count']) * 1000.0
            avg_pts = acc['points'] / acc['count']
            self.logger.info(
                "[PointCloud Profile][%s] npts≈%.0f project=%.1fms filter=%.1fms serialize=%.1fms",
                camera_name, avg_pts, avg_project, avg_filter, avg_serialize
            )
            acc.update({'project': 0.0, 'filter': 0.0, 'serialize': 0.0, 'points': 0, 'count': 0})

    def _parse_pointcloud_transform(self, cfg):
        """
        Parses camera-specific pointcloud transforms.

        Args:
            cfg (dict or None): Configuration entry that may define a `cameras` dict
                or a single transform for backward compatibility.

        Returns:
            dict: Mapping from camera name to (transform_matrix, is_local) tuples.
        """
        if cfg is None:
            return {}

        if isinstance(cfg, Mapping) and 'cameras' in cfg:
            per_camera_cfg = cfg.get('cameras') or {}
            per_camera = {}
            for cam_name, cam_cfg in per_camera_cfg.items():
                tf = self._build_transform(cam_cfg)
                if tf is not None:
                    per_camera[cam_name] = tf
            return per_camera

        # backwards compatibility: treat dict as single transform shared by all cameras
        tf = self._build_transform(cfg)
        return {'*': tf} if tf is not None else {}

    def _build_transform(self, cfg):
        if cfg is None:
            return None
        try:
            translation = np.array(cfg.get('translation', [0.0, 0.0, 0.0]), dtype=float).reshape(3)
        except Exception:
            translation = np.zeros(3, dtype=float)
        try:
            rpy_deg = np.array(cfg.get('rpy_deg', [0.0, 0.0, 0.0]), dtype=float).reshape(3)
        except Exception:
            rpy_deg = np.zeros(3, dtype=float)

        frame = str(cfg.get('frame', 'world')).lower()

        rot = self._rpy_to_matrix(np.deg2rad(rpy_deg))
        transform = np.eye(4, dtype=float)
        transform[:3, :3] = rot
        transform[:3, 3] = translation
        if np.allclose(transform, np.eye(4)):
            return None
        return (transform, frame == 'local')

    def _apply_pointcloud_transform(self, extr, camera_name):
        if extr is None:
            return None
        result = extr
        transforms = []
        cam_tf = self._pc_camera_tfs.get(camera_name)
        if cam_tf is not None:
            transforms.append(cam_tf)
        wildcard_tf = self._pc_camera_tfs.get('*')
        if wildcard_tf is not None and cam_tf is None:
            transforms.append(wildcard_tf)
        if not transforms:
            return extr
        try:
            for tf, is_local in transforms:
                if is_local:
                    result = result @ tf
                else:
                    result = tf @ result
            return result
        except Exception:
            return extr

    def _publish_camera_tf(self, camera_name, extrinsics, stamp_sec=None, stamp_nsec=None):
        if self.tf_broadcaster is None or extrinsics is None:
            return
        try:
            msg = TransformStamped()
            if stamp_sec is not None:
                msg.header.stamp = RosTime(sec=int(stamp_sec), nanosec=int(stamp_nsec))
            else:
                msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = self.tf_parent_frame
            msg.child_frame_id = camera_name
            msg.transform.translation.x = float(extrinsics[0, 3])
            msg.transform.translation.y = float(extrinsics[1, 3])
            msg.transform.translation.z = float(extrinsics[2, 3])
            quat = self._matrix_to_quaternion(extrinsics[:3, :3])
            msg.transform.rotation.x = quat[0]
            msg.transform.rotation.y = quat[1]
            msg.transform.rotation.z = quat[2]
            msg.transform.rotation.w = quat[3]
            self.tf_broadcaster.sendTransform(msg)
        except Exception:
            pass

    def _publish_rendered_camera_transforms(self, stamp_sec, stamp_nsec):
        """Publish camera TF independently from optional point-cloud output."""
        for camera_name in (self.camera_1_name, self.camera_2_name):
            if camera_name is None:
                continue
            extrinsics = self.sim.get_camera_extrinsics(camera_name)
            extrinsics = self._apply_pointcloud_transform(
                extrinsics,
                camera_name,
            )
            self._publish_camera_tf(
                camera_name,
                extrinsics,
                stamp_sec,
                stamp_nsec,
            )

    @staticmethod
    def _rpy_to_matrix(rpy):
        """Convert roll/pitch/yaw (rad) into a rotation matrix (Rz * Ry * Rx)."""
        roll, pitch, yaw = rpy
        cx, sx = np.cos(roll), np.sin(roll)
        cy, sy = np.cos(pitch), np.sin(pitch)
        cz, sz = np.cos(yaw), np.sin(yaw)

        Rx = np.array([[1, 0, 0],
                       [0, cx, -sx],
                       [0, sx, cx]], dtype=float)
        Ry = np.array([[cy, 0, sy],
                       [0, 1, 0],
                       [-sy, 0, cy]], dtype=float)
        Rz = np.array([[cz, -sz, 0],
                       [sz, cz, 0],
                       [0, 0, 1]], dtype=float)
        return Rz @ Ry @ Rx

    @staticmethod
    def _matrix_to_quaternion(rot):
        """Convert a 3x3 rotation matrix into (x, y, z, w) quaternion."""
        m = rot
        trace = m[0, 0] + m[1, 1] + m[2, 2]
        if trace > 0.0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (m[2, 1] - m[1, 2]) * s
            y = (m[0, 2] - m[2, 0]) * s
            z = (m[1, 0] - m[0, 1]) * s
        else:
            if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
                s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
                w = (m[2, 1] - m[1, 2]) / s
                x = 0.25 * s
                y = (m[0, 1] + m[1, 0]) / s
                z = (m[0, 2] + m[2, 0]) / s
            elif m[1, 1] > m[2, 2]:
                s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
                w = (m[0, 2] - m[2, 0]) / s
                x = (m[0, 1] + m[1, 0]) / s
                y = 0.25 * s
                z = (m[1, 2] + m[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
                w = (m[1, 0] - m[0, 1]) / s
                x = (m[0, 2] + m[2, 0]) / s
                y = (m[1, 2] + m[2, 1]) / s
                z = 0.25 * s
        return np.array([x, y, z, w], dtype=float)


    def _execute_traj_goal(self, goal_handle):
        """Execute callback for trajectory action (FollowJointTrajectory).

        Stores the incoming JointTrajectory into the internal _active_traj so the
        simulation loop will step it. Blocks until trajectory completes.
        """
        self.logger.info("Received trajectory goal")
        req = goal_handle.request
        traj = getattr(req, 'trajectory', None)
        if traj is None or len(traj.points) == 0:
            self.logger.warning("Rejecting trajectory goal: empty trajectory")
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "Trajectory has no points"
            return result

        # store a simple dict matching the previous roslibpy message shape
        with self._lock:
            self._active_traj = {'points': traj.points, 'joint_names': traj.joint_names}
            self._traj_start_time = time.perf_counter()
            self._traj_idx = 0
            
        self.logger.info("Trajectory goal accepted with %d points", len(traj.points))

        # wait for sim loop to clear the active trajectory (completion)
        while True:
            time.sleep(0.02)
            with self._lock:
                if self._active_traj is None:
                    break
                # allow cancellation if needed in future

        goal_handle.succeed()
        return FollowJointTrajectory.Result()

    def _handle_reset_sim(self, request, response):
        """Handle default simulation reset service.
        
        Resets to the initial configuration defined in MuJoCoInterface (self.sim.q0, etc.)
        """
        try:
            with self._lock:
                # Reset to default initial state
                self.sim.reset(qpos=self.sim.q0, qvel=self.sim.vel0, ctrl=self.sim.ctrl0)
                
                # Reset controller targets
                self._arm_target_pos = np.copy(self.sim.data.ctrl)[:self._arm_dof]
                self._active_traj = None
                
                # Reset gripper adapter
                self.gripper_adapter.set_current_position(0.0)
                
            response.success = True
            response.message = "Simulation reset to default configuration"
            self.logger.info("Simulation reset to default")
            
        except Exception as e:
            response.success = False
            response.message = f"Reset failed: {str(e)}"
            self.logger.error(f"Reset service failed: {e}")
            
        return response

    def _handle_reset_sim_custom(self, request, response):
        """Handle custom simulation reset service.
        
        Reads the 'reset_spec' parameter (JSON string) with the following format:
        {
            "robot": {
                "joint_positions": [j1, j2, j3, j4, j5, j6]  // optional
            },
            "objects": {
                "object_name": {
                    "position": [x, y, z],      // optional
                    "orientation": [w, x, y, z] // optional (quaternion)
                },
                ...
            }
        }
        """
        try:
            # Get reset specification from parameter
            reset_spec_str = self.node.get_parameter('reset_spec').get_parameter_value().string_value
            
            print(f"Custom reset specification: {reset_spec_str}")
            
            if reset_spec_str.startswith("string:"):
                reset_spec_str = reset_spec_str[len("string:"):]
            
            if not reset_spec_str or reset_spec_str == '{}':
                # Fall back to default reset if no custom spec
                return self._handle_reset_sim(request, response)
            
            reset_spec = json.loads(reset_spec_str)
            
            with self._lock:
                # First, reset to default state
                self.sim.reset(qpos=self.sim.q0, qvel=self.sim.vel0, ctrl=self.sim.ctrl0)
                
                # Apply custom end-effector configuration
                if 'robot' in reset_spec:
                    robot_spec = reset_spec['robot']
                    if 'joint_positions' in robot_spec:
                        joint_pos = np.array(robot_spec['joint_positions'], dtype=float)
                        
                        # Validate joint positions
                        if not np.all(np.isfinite(joint_pos)):
                            self.logger.error(f"Joint positions contain NaN or Inf: {joint_pos}")
                            raise ValueError("Invalid joint positions: contains NaN or Inf")
                        
                        if len(joint_pos) == self._arm_dof:
                            # Check joint limits if available
                            if self.sim.model.jnt_limited is not None:
                                for i in range(min(self._arm_dof, len(self.sim.model.jnt_range))):
                                    if self.sim.model.jnt_limited[i]:
                                        jnt_min, jnt_max = self.sim.model.jnt_range[i]
                                        if not (jnt_min <= joint_pos[i] <= jnt_max):
                                            self.logger.warning(f"Joint {i} position {joint_pos[i]} outside limits [{jnt_min}, {jnt_max}]")
                            
                            # Update qpos for arm joints
                            self.sim.data.qpos[:self._arm_dof] = joint_pos
                            # Update control targets
                            self.sim.data.ctrl[:self._arm_dof] = joint_pos
                            self._arm_target_pos = joint_pos.copy()
                            self.logger.debug(f"Applied robot joint positions: {joint_pos}")
                        else:
                            self.logger.warning(f"Invalid joint_positions length: {len(joint_pos)}, expected {self._arm_dof}")
                
                # Apply custom object poses
                if 'objects' in reset_spec:
                    for obj_name, obj_spec in reset_spec['objects'].items():
                        position = obj_spec.get('position', None)
                        orientation = obj_spec.get('orientation', None)
                        
                        # Validate position
                        if position is not None:
                            pos_array = np.array(position, dtype=float)
                            if not np.all(np.isfinite(pos_array)):
                                self.logger.error(f"Object {obj_name} position contains NaN or Inf: {position}")
                                continue
                            if len(pos_array) != 3:
                                self.logger.error(f"Object {obj_name} position must have 3 elements, got {len(pos_array)}")
                                continue
                            # Warn about suspicious positions
                            if pos_array[2] < -0.1:  # z-coordinate below table
                                self.logger.warning(f"Object {obj_name} has negative z-position: {pos_array[2]} (below table?)")
                        
                        # Validate orientation
                        if orientation is not None:
                            quat_array = np.array(orientation, dtype=float)
                            if not np.all(np.isfinite(quat_array)):
                                self.logger.error(f"Object {obj_name} orientation contains NaN or Inf: {orientation}")
                                continue
                            if len(quat_array) != 4:
                                self.logger.error(f"Object {obj_name} orientation must have 4 elements, got {len(quat_array)}")
                                continue
                            # Check if quaternion is valid
                            norm = np.linalg.norm(quat_array)
                            if norm < 1e-8:
                                self.logger.error(f"Object {obj_name} has zero-norm quaternion")
                                continue
                        
                        self.logger.debug(f"Setting pose for object {obj_name}: pos={position}, quat={orientation}")
                        success = self.sim.set_object_pose(obj_name, position, orientation)
                        if not success:
                            self.logger.warning(f"Failed to set pose for object: {obj_name}")
                
                # Validate state before forward kinematics
                if not np.all(np.isfinite(self.sim.data.qpos)):
                    self.logger.error("qpos contains NaN or Inf before mj_forward")
                    raise ValueError("Invalid qpos state")
                if not np.all(np.isfinite(self.sim.data.qvel)):
                    self.logger.error("qvel contains NaN or Inf before mj_forward")
                    raise ValueError("Invalid qvel state")
                
                # Forward kinematics to update derived quantities
                try:
                    mujoco.mj_forward(self.sim.model, self.sim.data)
                except Exception as e:
                    self.logger.error(f"mj_forward failed: {e}")
                    raise
                
                # Clear trajectories
                self._active_traj = None
                
                # Reset gripper
                self.gripper_adapter.set_current_position(0.0)
                
            response.success = True
            response.message = f"Simulation reset with custom configuration"
            self.logger.info(f"Custom reset applied: {reset_spec_str}")
            
        except json.JSONDecodeError as e:
            response.success = False
            response.message = f"Invalid JSON in reset_spec parameter: {str(e)}"
            self.logger.error(f"JSON decode error in reset_spec: {e}")
        except Exception as e:
            response.success = False
            response.message = f"Custom reset failed: {str(e)}"
            self.logger.error(f"Custom reset service failed: {e}")
            
        return response

    def _sim_loop(self):
        """Run physics near real-time; control, joint-states and cameras on separate wall-time cadences."""
        control_dt = float(self.sim.ctr_timestep)              # command update period
        physics_dt = float(self.sim.model.opt.timestep)        # MuJoCo step
        max_substeps = 100                                     # enough to keep up, but bounded

        next_ctrl_t = time.perf_counter() + control_dt
        next_js_t = time.perf_counter() + self.publish_interval
        next_cam_t = time.perf_counter() + self._cam_period

        prealloc_c1 = None
        prealloc_c2 = None
        prealloc_d1 = None
        prealloc_d2 = None
        if self._cam_size:
            w, h = self._cam_size
            prealloc_c1 = np.empty((h, w, 3), dtype=np.uint8)
            prealloc_c2 = np.empty((h, w, 3), dtype=np.uint8)
            # Pre-allocate depth buffers
            prealloc_d1 = np.empty((h, w), dtype=np.float32)
            prealloc_d2 = np.empty((h, w), dtype=np.float32)

        self.sim.init_renderer(height=self._cam_size[1], width=self._cam_size[0])

        self.logger.info("ctr_timestep=%f", self.sim.ctr_timestep)
        self.logger.info("model.opt.timestep=%f", self.sim.model.opt.timestep)

        acc = 0.0
        t_last = time.perf_counter()
        last_ctrl_target = np.zeros(self.nu, dtype=float)

        while self._running:
            now = time.perf_counter()
            elapsed = now - t_last
            t_last = now
            acc += elapsed

            loop_t0 = time.perf_counter()

            # ---- control target update on wall-clock cadence ----
            while now >= next_ctrl_t:
                with self._lock:
                    if self._active_traj is not None:
                        t_ref = self._traj_start_time if self._traj_start_time is not None else time.perf_counter()
                        t_now = max(0.0, time.perf_counter() - t_ref)
                        points = self._active_traj.get('points', [])
                        joint_names = self._active_traj.get('joint_names', [])

                        if points:
                            while (
                                self._traj_idx + 1 < len(points)
                                and self._ros_time_from_start(points[self._traj_idx + 1]) <= t_now
                            ):
                                self._traj_idx += 1

                            pt = points[self._traj_idx]
                            arm_pos = np.array(pt.positions, dtype=float)
                            arm_pos = self._order_joint_positions(joint_names, arm_pos)
                            if arm_pos.shape[0] == self._arm_dof:
                                self._arm_target_pos = arm_pos.copy()

                            if (
                                self._traj_idx == len(points) - 1
                                and t_now > self._ros_time_from_start(pt)
                            ):
                                self._active_traj = None
                                self._traj_idx = 0
                    elif self._traj_start_time is not None:
                        self._traj_start_time = None

                    last_ctrl_target[:].fill(0.0)
                    last_ctrl_target[:self._arm_dof] = self._arm_target_pos
                    last_ctrl_target[-1] = self.gripper_adapter.get_target_position()

                next_ctrl_t += control_dt

            # ---- physics stepping: advance sim-time to match wall-time ----
            # NOTE: lock must be held around every mj_step call.
            # The reset service handler (and any other writer of sim.data) holds
            # this same lock, so grabbing it here guarantees no concurrent write
            # can corrupt MuJoCo's internal state (e.g. SAP broadphase arrays)
            # while a physics step is in progress.
            phys_t0 = time.perf_counter()
            substeps = 0
            while acc >= physics_dt and substeps < max_substeps:
                with self._lock:
                    self.sim.apply_joint_action(last_ctrl_target)
                    self.sim.step_simulation()
                acc -= physics_dt
                substeps += 1

            # avoid unbounded backlog; keep only up to one control period
            if acc > control_dt:
                acc = control_dt

            phys_ms = (time.perf_counter() - phys_t0) * 1000.0

            # ---- joint states ----
            js_ms = 0.0
            now = time.perf_counter()
            if now >= next_js_t:
                t0 = time.perf_counter()

                prop = self.sim.get_proprioception()
                self.gripper_adapter.set_current_position(
                    prop.get('qpos', [0.0] * self.nu)[self._gripper_joint_id]
                )

                positions, velocities = [], []
                for jid in self._joint_ids:
                    if jid is None or jid < 0:
                        positions.append(0.0)
                        velocities.append(0.0)
                    else:
                        positions.append(float(prop['qpos'][jid]))
                        try:
                            velocities.append(float(prop['qvel'][jid]))
                        except Exception:
                            velocities.append(0.0)

                js = JointState()
                js.header = Header()
                js.header.stamp = self.node.get_clock().now().to_msg()
                js.name = self.publisher_joint_order
                js.position = positions
                js.velocity = velocities
                js.effort = [0.0] * len(self.publisher_joint_order)

                try:
                    self.js_pub.publish(js)
                except Exception as e:
                    self.logger.warning("Failed to publish joint_states: %s", str(e))

                try:
                    scene_markers = self._create_scene_marker_array()
                    self.scene_pub.publish(scene_markers)
                except Exception as e:
                    self.logger.warning("Failed to publish scene description: %s", str(e))

                self._publish_scene_clearance_bounds()

                while next_js_t <= now:
                    next_js_t += self.publish_interval

                js_ms = (time.perf_counter() - t0) * 1000.0

            # ---- cameras ----
            cam_ms = 0.0
            now = time.perf_counter()
            if now >= next_cam_t:
                t0 = time.perf_counter()

                cam1_img = None
                cam2_img = None
                depth1 = None
                depth2 = None
                stamp_now = time.time()

                # Use batch rendering for maximum efficiency
                camera_configs = []
                
                if self.camera_1_name is not None:
                    need_depth = self._publish_pc_cam1 or self._publish_depth_cam1
                    camera_configs.append({
                        'name': self.camera_1_name,
                        'out_color': prealloc_c1,
                        'out_depth': prealloc_d1 if need_depth else None,
                        'need_depth': need_depth
                    })
                
                if self.camera_2_name is not None:
                    need_depth = self._publish_pc_cam2 or self._publish_depth_cam2
                    camera_configs.append({
                        'name': self.camera_2_name,
                        'out_color': prealloc_c2,
                        'out_depth': prealloc_d2 if need_depth else None,
                        'need_depth': need_depth
                    })
                
                # Render all cameras in one optimized batch
                if camera_configs:
                    results = self.sim.render_cameras_batch(camera_configs)
                    if len(results) >= 1:
                        cam1_img, depth1 = results[0]
                    if len(results) >= 2:
                        cam2_img, depth2 = results[1]

                self._enqueue_image_pair(
                    cam1_img, cam2_img, depth_1=depth1, depth_2=depth2, stamp=stamp_now
                )
                sec = int(stamp_now)
                nsec = int((stamp_now % 1) * 1e9)
                self._publish_rendered_camera_transforms(sec, nsec)
                stamp_perf = time.perf_counter()
                if stamp_perf >= self._next_pc_enqueue_t:
                    self._enqueue_pointclouds(sec, nsec, cam1_img, depth1, cam2_img, depth2)
                    while self._next_pc_enqueue_t <= stamp_perf:
                        self._next_pc_enqueue_t += self._pc_period

                while next_cam_t <= now:
                    next_cam_t += self._cam_period

                cam_ms = (time.perf_counter() - t0) * 1000.0

            loop_ms = (time.perf_counter() - loop_t0) * 1000.0
            if loop_ms > 100.0: # only log if the loop is actually doing significant work
                self.logger.info(
                    "loop=%.1f ms phys=%.1f js=%.1f cam=%.1f substeps=%d acc=%.4f",
                    loop_ms, phys_ms, js_ms, cam_ms, substeps, acc
                )

            next_wakeup = min(next_ctrl_t, next_js_t, next_cam_t)
            sleep_time = next_wakeup - time.perf_counter()
            if sleep_time > 0:
                time.sleep(min(sleep_time, 0.002))

    def _create_scene_marker_array(self):
        """Create a MarkerArray message from the current scene description."""
        marker_array = MarkerArray()
        scene_objects = self.sim.get_scene_description()
        
        for i, obj_info in enumerate(scene_objects):
            marker = Marker()
            marker.header = Header()
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.header.frame_id = "base_link"  # or base_link depending on your setup
            marker.ns = "scene_objects"
            marker.id = i
            marker.action = Marker.ADD
            
            # Set marker type based on object type
            if obj_info['type'] == 'box':
                marker.type = Marker.CUBE
                marker.scale = Vector3(x=obj_info['size'][0]*2, y=obj_info['size'][1]*2, z=obj_info['size'][2]*2)
            elif obj_info['type'] == 'sphere':
                marker.type = Marker.SPHERE
                marker.scale = Vector3(x=obj_info['size'][0]*2, y=obj_info['size'][0]*2, z=obj_info['size'][0]*2)
            elif obj_info['type'] == 'cylinder':
                marker.type = Marker.CYLINDER
                marker.scale = Vector3(x=obj_info['size'][0]*2, y=obj_info['size'][0]*2, z=obj_info['size'][1]*2)
            else:
                marker.type = Marker.CUBE  # default fallback
                marker.scale = Vector3(x=0.1, y=0.1, z=0.1)
            
            # Set position and orientation
            marker.pose = Pose()
            marker.pose.position = Point(x=obj_info['position'][0], y=obj_info['position'][1], z=obj_info['position'][2])
            marker.pose.orientation = Quaternion(w=obj_info['orientation'][0], x=obj_info['orientation'][1], 
                                                y=obj_info['orientation'][2], z=obj_info['orientation'][3])
            
            # Set color
            color = obj_info['color']
            marker.color.r = color[0]
            marker.color.g = color[1] 
            marker.color.b = color[2]
            marker.color.a = color[3]
            
            # Marker lifetime
            marker.lifetime.sec = 0  # persistent marker
            marker.lifetime.nanosec = 0
            
            marker.text = obj_info['name']
            
            marker_array.markers.append(marker)
        
        return marker_array

    def _create_scene_clearance_bounds_marker_array(self, bounds, stamp):
        """Build one complete world-axis CUBE MarkerArray from live bounds."""
        marker_array = MarkerArray()
        for index, bound in enumerate(bounds):
            name = str(getattr(bound, 'name', '') or '').strip()
            center = np.asarray(getattr(bound, 'center', None), dtype=float)
            size = np.asarray(getattr(bound, 'size', None), dtype=float)
            if not name:
                raise ValueError(f"Clearance bound {index} has no object name")
            if center.shape != (3,) or not np.all(np.isfinite(center)):
                raise ValueError(
                    f"Clearance bound {name!r} has invalid center: {center}"
                )
            if (
                size.shape != (3,)
                or not np.all(np.isfinite(size))
                or np.any(size <= 0.0)
            ):
                raise ValueError(
                    f"Clearance bound {name!r} has invalid size: {size}"
                )

            marker = Marker()
            marker.header = Header()
            marker.header.stamp = stamp
            marker.header.frame_id = 'base_link'
            marker.ns = 'scene_clearance_bounds'
            marker.id = index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = Pose()
            marker.pose.position = Point(
                x=float(center[0]),
                y=float(center[1]),
                z=float(center[2]),
            )
            marker.pose.orientation = Quaternion(
                x=0.0,
                y=0.0,
                z=0.0,
                w=1.0,
            )
            marker.scale = Vector3(
                x=float(size[0]),
                y=float(size[1]),
                z=float(size[2]),
            )
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
            marker.color.a = 0.25
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0
            marker.text = name
            marker_array.markers.append(marker)
        return marker_array

    def _record_scene_clearance_profile_sample(self, elapsed_ms, object_count):
        samples = getattr(self, '_scene_clearance_profile_samples', None)
        if samples is None:
            return
        samples.append(float(elapsed_ms))
        if len(samples) < 100:
            return
        sample_array = np.asarray(samples, dtype=float)
        self.logger.info(
            "Scene clearance bounds profile: objects=%d samples=%d "
            "median=%.3fms p95=%.3fms max=%.3fms",
            int(object_count),
            len(samples),
            float(np.median(sample_array)),
            float(np.percentile(sample_array, 95.0)),
            float(np.max(sample_array)),
        )
        self._scene_clearance_profile_samples = None

    def _log_scene_clearance_publish_error(self, error):
        now = time.monotonic()
        last_log = getattr(
            self,
            '_last_scene_clearance_error_log_time',
            float('-inf'),
        )
        if now - last_log < 2.0:
            return
        self._last_scene_clearance_error_log_time = now
        self.logger.error(
            "Scene clearance bounds array not published this cycle; "
            "complete bounds are unavailable: %s",
            str(error),
        )

    def _publish_scene_clearance_bounds(self):
        """Publish all live bounds atomically, or publish nothing on failure."""
        try:
            start_time = time.perf_counter()
            bounds = self.sim.get_scene_clearance_bounds()
            elapsed_ms = (time.perf_counter() - start_time) * 1000.0
            stamp = self.node.get_clock().now().to_msg()
            marker_array = self._create_scene_clearance_bounds_marker_array(
                bounds,
                stamp,
            )
            self.scene_clearance_pub.publish(marker_array)
            self._record_scene_clearance_profile_sample(elapsed_ms, len(bounds))
            return True
        except Exception as error:
            self._log_scene_clearance_publish_error(error)
            return False

    def _order_joint_positions(self, joint_names, pos):
        # sort positions according to publisher_joint_order
        pos_sorted = np.zeros(len(self.publisher_joint_order), dtype=float)
        for i, jn in enumerate(joint_names):
            if jn in self.publisher_joint_order:
                idx = self.publisher_joint_order.index(jn)
                pos_sorted[idx] = pos[i]
        pos = pos_sorted
        return pos

    def _ros_time_from_start(self, pt):
        # pt: dict with 'time_from_start' as {'secs': int, 'nsecs': int}
        if isinstance(pt, JointTrajectoryPoint):
            return pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
        elif isinstance(pt, dict) and 'time_from_start' in pt:
            tfs = pt['time_from_start']
        else:
            tfs = {'secs': 0, 'nsecs': 0}
        return tfs.get('secs', 0) + tfs.get('nsecs', 0) * 1e-9

    def close(self):
        self._running = False
        self._img_pub_running = False
        self._pcd_pub_running = False
        self._thread.join()
        try:
            self.sim.close()
        except Exception:
            pass
        # cleanup rclpy resources
        try:
            self._traj_action_server.destroy()
        except Exception:
            pass
        try:
            self._reset_service.destroy()
        except Exception:
            pass
        try:
            self._reset_custom_service.destroy()
        except Exception:
            pass
        try:
            self.camera_timer.destroy()
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        try:
            self._img_pub_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if hasattr(self, '_pcd_pub_thread') and self._pcd_pub_thread.is_alive():
                self._pcd_pub_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            # join spin thread
            if hasattr(self, '_rclpy_thread') and self._rclpy_thread.is_alive():
                self._rclpy_thread.join(timeout=1.0)
        except Exception:
            pass

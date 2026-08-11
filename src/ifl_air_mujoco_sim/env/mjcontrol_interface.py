import mujoco
import mujoco.viewer
import numpy as np
import xml.etree.ElementTree as ET
import time
from dm_control import mjcf
import tempfile
import os
import copy
from env.utils.populate_scene import (
    assign_placement_slots,
    normalize_scene_mode,
    populate_scene,
    select_scene_objects_for_mode,
)
from env.utils.scene_clearance_bounds import (
    build_object_geometry_cache,
    compute_world_aabb,
)
from env.utils.sorting_scene import validate_sorting_camera_visibility

class MuJoCoInterface:
    def __init__(self,
                model_path,
                camera_names=['cam0'],
                headless=True,
                control_timestep=0.01,
                render_fps=30,
                objects_config=None,
                random_object_count=0,
                fixed_object_names=None,
                scene_mode='mix',
                scene_object_categories=None,
                assigned_object_names=None,
                placement_slots=None,
                classification_bins=None,
                camera_size=(1280, 720),
                minimal_viewer=False, # if true, hides extra UI elements and visual overlays for a cleaner view (good for recording videos)
                viewer_sync_fps=60):

        self.renderer = None
        self.objects_config = objects_config or []
        self.random_object_count = random_object_count
        self.fixed_object_names = fixed_object_names or []
        self.scene_mode = normalize_scene_mode(scene_mode)
        self.scene_object_categories = (
            scene_object_categories
            if scene_object_categories is not None
            else {}
        )
        self.assigned_object_names = assigned_object_names or []
        self.placement_slots = placement_slots or []
        self.classification_bins = classification_bins
        self.camera_names = list(camera_names or ['cam0'])
        self.camera_size = tuple(int(value) for value in camera_size)
        if len(self.camera_size) != 2 or min(self.camera_size) <= 0:
            raise ValueError(
                "camera_size must be [positive_width, positive_height]."
            )
        self._renderer_width = None
        self._renderer_height = None

        # Select before populate_scene so n_objects and scene markers match the model.
        self.objects_config = select_scene_objects_for_mode(
            self.objects_config,
            scene_mode=self.scene_mode,
            random_object_count=self.random_object_count,
            fixed_object_names=self.fixed_object_names,
            scene_object_categories=self.scene_object_categories,
            assigned_object_names=self.assigned_object_names,
        )
        if self.placement_slots:
            self.objects_config = assign_placement_slots(
                self.objects_config,
                self.placement_slots,
            )
        selected_names = ", ".join(
            obj['name'] for obj in self.objects_config
        )
        print(
            f"[INFO] Scene mode {self.scene_mode}; "
            f"selected objects by slot: {selected_names}"
        )
        if self.scene_mode == 'mix' and self.fixed_object_names:
            selected_names = ", ".join(obj['name'] for obj in self.objects_config)
            print(f"[INFO] Selected fixed-priority scene objects: {selected_names}")
        elif self.scene_mode == 'mix' and self.random_object_count > 0:
            print(f"[INFO] Randomly selected {len(self.objects_config)} objects from pool")

        print("[INFO] STARTING MUJOCO SCENE NOW")
        populated_scene = populate_scene(
            model_path,
            objects_config=self.objects_config,
            camera_names=self.camera_names,
            classification_bins=self.classification_bins,
        )
        self.model = mujoco.MjModel.from_xml_string(populated_scene)
        self.data = mujoco.MjData(self.model)
        self._initialize_scene_clearance_geometry_cache()

        self.ctr_timestep = control_timestep  # In seconds (e.g., 0.01 = 100Hz)
        self.default_cam_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_CAMERA,
            self.camera_names[0],
        )
        
        self.viewer = None
        self._viewer_sync_fps = max(1, int(viewer_sync_fps))
        self._viewer_sync_period = 1.0 / self._viewer_sync_fps
        self._next_viewer_sync_t = time.perf_counter()

        if not headless:
            self.viewer = mujoco.viewer.launch_passive(
                self.model,
                self.data,
                show_left_ui=not minimal_viewer,
                show_right_ui=not minimal_viewer,
            )
            print("[INFO] Viewer launched")

            if minimal_viewer:
                with self.viewer.lock():
                    # turn off extra visual overlays
                    self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 0
                    self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = 0
                    self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = 0
                    self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_ACTUATOR] = 0
                    self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CAMERA] = 0
                    self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_PERTFORCE] = 0
                    self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_PERTOBJ] = 0
        
        # Viewer cadence: sync only every N sim steps to avoid throttling control loop
        self._render_fps = max(1, int(render_fps))
        self._steps_per_render = max(1, int(round((1.0 / self._render_fps) / self.model.opt.timestep)))
        self._render_counter = 0
        self._prev_render_time = time.perf_counter()
        
        self._camera_id_cache = {}
        for cam_name in self.camera_names:
            cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, cam_name)
            if cam_id != -1:
                self._camera_id_cache[cam_name] = cam_id
        
        # Track scene version to avoid redundant updates
        self._scene_version = 0
        self._last_rendered_version = -1

        # Number of position, velocity and control variables
        self.nq = self.model.nq
        self.nv = self.model.nv
        self.nu = self.model.nu

        print(f"[INFO] MuJoCo model loaded with {self.nq} position, {self.nv} velocity and {self.nu} control variables.")
        self.n_objects = len(self.objects_config)
        print(f"[INFO] Scene contains {self.n_objects} objects.")

        # Use model's qpos0 (set by populate_scene) as initial state.
        # Only override the robot arm joints; object poses already match the XML.
        self.q0 = self.data.qpos.copy()
        q0_ur10 = [-0.2345, -1.0715, -1.8688, -1.5812, 1.6339, 2.8947]
        arm_dof = len(q0_ur10)
        self.q0[:arm_dof] = q0_ur10

        self.vel0 = [0] * self.nv  # Zero velocity
        #self.ctrl0 = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0, 0]
        self.ctrl0 = [-0.2345, -1.0715, -1.8688, -1.5812, 1.6339, 2.8947, 0]
        self.reset(qpos=self.q0, qvel=self.vel0, ctrl=self.ctrl0)
        if self.classification_bins and self.classification_bins.get(
            'enabled', False
        ):
            if 'camera_orbbec' not in self.camera_names:
                raise ValueError(
                    "Sorting mode requires camera_orbbec as overview camera."
                )
            validate_sorting_camera_visibility(
                self.model,
                self.data,
                'camera_orbbec',
                self.objects_config,
                self.classification_bins,
                image_size=self.camera_size,
            )

    def _sync_viewer_if_needed(self, force=False, state_only=True):
        if self.viewer is None or not self.viewer.is_running():
            return

        now = time.perf_counter()
        if force or now >= self._next_viewer_sync_t:
            with self.viewer.lock():
                self.viewer.sync(state_only=state_only)
            while self._next_viewer_sync_t <= now:
                self._next_viewer_sync_t += self._viewer_sync_period
    
    def init_renderer(self, width=640, height=480, max_geom=None):
        """ Initialize the renderer for camera images with GPU acceleration.
        Must be called in the thread which will be used to call get_camera_image()
        
        Args:
            width: Image width
            height: Image height
            max_geom: Maximum number of geometries (None = auto)
        """
        self._renderer_width = width
        self._renderer_height = height
        
        # Use OpenGL backend for GPU rendering (much faster)
        # max_geom limits geometry buffer size for better performance
        if max_geom is None:
            max_geom = min(self.model.ngeom * 2, 10000)  # Reasonable default
        
        try:
            self.renderer = mujoco.Renderer(
                self.model, 
                height=height, 
                width=width,
                max_geom=max_geom
            )
            print(f"[INFO] Renderer initialized: {width}x{height}, max_geom={max_geom}, GPU-accelerated")
        except Exception as e:
            # Fallback without max_geom if not supported
            self.renderer = mujoco.Renderer(self.model, height=height, width=width)
            print(f"[INFO] Renderer initialized: {width}x{height}, standard mode")

    
    def _get_keyframe_index_by_name(self, name):
        # Get the index of the keyframe by name
        for i in range(self.model.nkey):
            if mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_KEY, i) == name:
                return i
        raise ValueError(f"Keyframe '{name}' not found.")
    
    

    def reset(self, qpos=None, qvel=None, ctrl=None):
        """Reset the simulation to given or default state."""
        if self.viewer and self.viewer.is_running():
            with self.viewer.lock():
                mujoco.mj_resetData(self.model, self.data)
                if qpos is not None:
                    self.data.qpos[:] = qpos
                if qvel is not None:
                    self.data.qvel[:] = qvel
                if ctrl is not None:
                    self.data.ctrl[:] = ctrl

                mujoco.mj_forward(self.model, self.data)
                self.data.cfrc_ext[:] = 0
                self.data.qfrc_applied[:] = 0

                for _ in range(10):
                    mujoco.mj_step(self.model, self.data)

                self._sync_viewer_if_needed(force=True, state_only=False)
        else:
            mujoco.mj_resetData(self.model, self.data)
            if qpos is not None:
                self.data.qpos[:] = qpos
            if qvel is not None:
                self.data.qvel[:] = qvel
            if ctrl is not None:
                self.data.ctrl[:] = ctrl

            mujoco.mj_forward(self.model, self.data)
            self.data.cfrc_ext[:] = 0
            self.data.qfrc_applied[:] = 0

            for _ in range(10):
                mujoco.mj_step(self.model, self.data)

    def step_simulation(self):
        if self.viewer and self.viewer.is_running():
            with self.viewer.lock():
                mujoco.mj_step(self.model, self.data)
        else:
            mujoco.mj_step(self.model, self.data)

        self._scene_version += 1
        self._sync_viewer_if_needed(force=False, state_only=True)


    def close(self):
        """Clean up resources."""
        if self.viewer:
            if self.viewer.is_running():
                self.viewer.close()
        # Remove any temporary XML file if it was created
        try:
            if hasattr(self, "tmp_xml_file") and self.tmp_xml_file is not None:
                os.remove(self.tmp_xml_file.name)
        except Exception:
            pass
        


    ### --- Action Interfaces ---

    def apply_joint_action(self, action):
        """Applies control action in joint space (e.g., torques or positions)."""
        assert len(action) == self.nu
        self.data.ctrl[:] = action

    def apply_task_space_delta(self, delta_pos, site_name, gains=1.0):
        """Move end-effector toward desired position (velocity-control)."""
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        J_pos = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, J_pos, None, site_id)

        dq = gains * np.linalg.pinv(J_pos).dot(delta_pos)
        self.data.qvel[:] = dq

    ### --- Sensors ---

    def _initialize_scene_clearance_geometry_cache(self):
        """Cache selected object geometry and compiled MuJoCo body IDs."""
        entries = []
        seen_names = set()
        for obj_config in self.objects_config:
            obj_name = str(obj_config.get('name') or '').strip()
            if not obj_name:
                raise RuntimeError("Scene clearance geometry requires object names")
            if obj_name in seen_names:
                raise RuntimeError(
                    f"Duplicate scene object name in clearance geometry cache: {obj_name!r}"
                )
            seen_names.add(obj_name)

            body_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_BODY,
                obj_name,
            )
            if body_id == -1:
                raise RuntimeError(
                    f"Could not find MuJoCo body {obj_name!r} for scene clearance bounds"
                )
            try:
                geometry_cache = build_object_geometry_cache(obj_config)
            except Exception as exc:
                raise RuntimeError(
                    f"Could not build scene clearance geometry for object "
                    f"{obj_name!r}: {exc}"
                ) from exc
            entries.append((obj_name, body_id, geometry_cache))

        self._scene_clearance_geometry_entries = tuple(entries)

    def get_scene_clearance_bounds(self):
        """Return complete live world-axis AABBs for all selected objects."""
        entries = getattr(self, '_scene_clearance_geometry_entries', ())
        expected_names = [
            str(obj_config.get('name') or '').strip()
            for obj_config in self.objects_config
        ]
        entry_names = [name for name, _, _ in entries]
        if entry_names != expected_names:
            raise RuntimeError(
                "Scene clearance geometry cache does not match selected objects: "
                f"expected={expected_names}, cached={entry_names}"
            )

        bounds = []
        for obj_name, body_id, geometry_cache in entries:
            try:
                body_position = self.data.xpos[body_id].copy()
                body_rotation = self.data.xmat[body_id].reshape(3, 3).copy()
                bound = compute_world_aabb(
                    geometry_cache,
                    body_position,
                    body_rotation,
                )
            except Exception as exc:
                raise RuntimeError(
                    f"Could not compute live scene clearance bound for object "
                    f"{obj_name!r}: {exc}"
                ) from exc
            bounds.append(bound)
        return bounds

    def get_proprioception(self):
        return {
            'qpos': np.copy(self.data.qpos),
            'qvel': np.copy(self.data.qvel),
            'ctrl': np.copy(self.data.ctrl),
        }
    
    def get_scene_description(self):
        """Get scene description with current object poses for ROS publishing.
        
        Returns:
            List of dictionaries containing object information for ROS MarkerArray
        """
        scene_objects = []
        
        for i, obj_config in enumerate(self.objects_config):
            obj_name = obj_config['name']
            
            # Find object body in MuJoCo model
            try:
                body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, obj_name)
                if body_id == -1:
                    continue
                
                # Get current position and orientation from simulation
                body_pos = self.data.xpos[body_id].copy()
                body_quat = self.data.xquat[body_id].copy()
                
                obj_info = {
                    'name': obj_name,
                    'type': obj_config['type'],
                    'size': obj_config['size'],
                    'position': body_pos.tolist(),
                    'orientation': body_quat.tolist(),  # [w, x, y, z]
                    'color': obj_config.get('color', [0.5, 0.5, 0.5, 1.0])
                }
                scene_objects.append(obj_info)
                
            except Exception as e:
                print(f"[WARNING] Could not find object {obj_name} in scene: {e}")
                continue
        
        return scene_objects
    
    def get_camera_image(self, camera_name=None, out=None):
        """Get camera image from specified camera or default camera.
        
        Args:
            camera_name: Name of the camera to render from. If None, uses default camera.
            
        Returns:
            RGB image as numpy array with shape (height, width, 3) or None if failed
        """
        # Check if renderer is available
        if self.renderer is None:
            print('[WARNING] Camera renderer not available (likely due to not being initialized)')
            return None
            
        try:
            if camera_name is None:
                cam_id = self.default_cam_id
            else:
                cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
                if cam_id == -1:
                    print(f'[WARNING] Camera {camera_name} not found, using default camera')
                    cam_id = self.default_cam_id
            
            # Update the scene with current data and camera
            self.renderer.update_scene(self.data, camera=cam_id)
            
            # Render the scene - modern MuJoCo API
            if out is None:
                img = self.renderer.render()
                return img
            else:
                self.renderer.render(out=out)
            
        except Exception as e:
            print(f'Failed to get camera image for {camera_name}: {e}')
            return None

    def render_camera(self, camera_name=None, out_color=None, out_depth=None, need_depth=False, update_scene=True):
        """
        Renders an RGB image and optional depth image from a specified camera.

        Args:
            camera_name (str, optional): Name of the camera. If None, the default camera is used.
            out_color (np.ndarray, optional): Optional buffer to write the RGB output into.
            out_depth (np.ndarray, optional): Optional buffer to write the depth output into.
            need_depth (bool, optional): If True, a depth image is rendered in addition to the RGB image.
            update_scene (bool, optional): If True, updates the scene before rendering. Set to False for batch rendering.

        Returns:
            tuple:
                color (np.ndarray or None): RGB image with shape (H, W, 3) or None if rendering failed.
                depth (np.ndarray or None): Depth image with shape (H, W) when requested, otherwise None.
        """
        if self.renderer is None:
            return None, None

        try:
            if camera_name is None:
                cam_id = self.default_cam_id
            else:
                cam_id = self._camera_id_cache.get(camera_name, self.default_cam_id)

            # Only update scene if requested and scene has changed
            if update_scene:
                self.renderer.update_scene(self.data, camera=cam_id)
                self._last_rendered_version = self._scene_version

            # Render RGB directly to buffer (GPU accelerated)
            if out_color is not None:
                self.renderer.render(out=out_color)
                color = out_color
            else:
                color = self.renderer.render()

            # Render depth if needed
            depth = None
            if need_depth:
                self.renderer.enable_depth_rendering()
                try:
                    if out_depth is not None:
                        self.renderer.render(out=out_depth)
                        depth = out_depth
                    else:
                        depth = self.renderer.render()
                finally:
                    self.renderer.disable_depth_rendering()

            return color, depth

        except Exception as e:
            print(f"Failed to render camera {camera_name}: {e}")
            return None, None
    
    def render_cameras_batch(self, camera_configs):
        """
        Render multiple cameras efficiently in a single batch.
        
        Args:
            camera_configs: List of dicts with keys:
                - 'name': camera name
                - 'out_color': output buffer for RGB
                - 'out_depth': output buffer for depth (optional)
                - 'need_depth': whether depth is needed
        
        Returns:
            List of (color, depth) tuples
        """
        if self.renderer is None or not camera_configs:
            return [(None, None)] * len(camera_configs)
        
        results = []
        
        try:
            # Render each camera with its own scene update to get the correct viewpoint
            for config in camera_configs:
                cam_name = config['name']
                out_color = config.get('out_color')
                out_depth = config.get('out_depth')
                need_depth = config.get('need_depth', False)
                
                # Each camera must update the scene so the renderer uses its viewpoint
                color, depth = self.render_camera(
                    cam_name,
                    out_color=out_color,
                    out_depth=out_depth,
                    need_depth=need_depth,
                    update_scene=True
                )
                results.append((color, depth))
            
            return results
            
        except Exception as e:
            print(f"Failed to render camera batch: {e}")
            return [(None, None)] * len(camera_configs)

    def get_camera_intrinsics(self, camera_name, width, height):
        """
        Computes a pinhole camera intrinsic matrix for a Mujoco camera.

        Args:
            camera_name (str): Name of the camera used to look up its Mujoco parameters.
            width (int): Image width in pixels.
            height (int): Image height in pixels.

        Returns:
            np.ndarray or None:
                A 3x3 intrinsic matrix of the form
                [[fx,  0, cx],
                [ 0, fy, cy],
                [ 0,  0,  1]]
                or None if the camera cannot be found or computation fails.
        """
        try:
            cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
            if cam_id == -1:
                print(f'[WARNING] Camera {camera_name} not found for intrinsics')
                return None

            fovy_deg = float(self.model.cam_fovy[cam_id])
            fovy_rad = np.deg2rad(fovy_deg)
            aspect = float(width) / float(height)
            fovx_rad = 2.0 * np.arctan(aspect * np.tan(fovy_rad / 2.0))

            fx = (width / 2.0) / np.tan(fovx_rad / 2.0)
            fy = (height / 2.0) / np.tan(fovy_rad / 2.0)
            cx = width / 2.0
            cy = height / 2.0

            return np.array(
                [[fx, 0.0, cx],
                 [0.0, fy, cy],
                 [0.0, 0.0, 1.0]],
                dtype=float
            )
        except Exception as e:
            print(f'[WARNING] Failed to compute intrinsics for {camera_name}: {e}')
            return None

    def get_camera_extrinsics(self, camera_name):
        """
        Computes the camera extrinsic matrix (camera pose in world coordinates).

        Args:
            camera_name (str): Name of the camera used to retrieve its Mujoco pose.

        Returns:
            np.ndarray or None:
                A 4x4 camera-to-world matrix where the upper 3x3 block is the
                rotation matrix and the last column contains the camera position.
                Returns None if the camera cannot be found or the computation fails.
        """
        try:
            cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
            if cam_id == -1:
                print(f'[WARNING] Camera {camera_name} not found for extrinsics')
                return None

            pos = self.data.cam_xpos[cam_id].copy()
            rot = self.data.cam_xmat[cam_id].reshape(3, 3).copy()

            extr = np.eye(4)
            extr[:3, :3] = rot
            extr[:3, 3] = pos
            return extr
        except Exception as e:
            print(f'[WARNING] Failed to compute extrinsics for {camera_name}: {e}')
            return None

    def set_object_pose(self, object_name, position=None, quaternion=None):
        """Set the pose of a free-floating object in the scene.
        
        Args:
            object_name: Name of the object body
            position: [x, y, z] position (optional)
            quaternion: [w, x, y, z] quaternion orientation (optional)
            
        Returns:
            True if successful, False otherwise
        """
        try:
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, object_name)
            if body_id == -1:
                print(f'[WARNING] Object {object_name} not found in scene')
                return False
            
            # Find the joint associated with this body (assumes free joint)
            if self.model.body_jntnum[body_id] == 0:
                return False
            joint_id = self.model.body_jntadr[body_id]
            if joint_id == -1:
                print(f'[WARNING] No joint found for object {object_name}')
                return False
            
            # Check joint type - must be free joint for 6-DOF objects
            joint_type = self.model.jnt_type[joint_id]
            if joint_type != mujoco.mjtJoint.mjJNT_FREE:
                print(f'[WARNING] Object {object_name} does not have a free joint (type={joint_type})')
                return False
            
            # Get qpos address for this joint
            qpos_addr = self.model.jnt_qposadr[joint_id]
            
            # Validate qpos address bounds
            if qpos_addr < 0 or qpos_addr + 7 > len(self.data.qpos):
                print(f'[ERROR] Invalid qpos address for {object_name}: {qpos_addr}')
                return False
            
            # Update position if provided (first 3 elements of free joint qpos)
            if position is not None:
                pos = np.asarray(position, dtype=float)
                if pos.shape != (3,) or not np.all(np.isfinite(pos)):
                    print(f'[ERROR] Invalid position for {object_name}: {position}')
                    return False
                self.data.qpos[qpos_addr:qpos_addr+3] = pos
            
            # Update quaternion if provided (next 4 elements of free joint qpos)
            if quaternion is not None:
                q = np.asarray(quaternion, dtype=float)
                if q.shape != (4,) or not np.all(np.isfinite(q)):
                    print(f'[ERROR] Invalid quaternion for {object_name}: {quaternion}')
                    return False
                n = np.linalg.norm(q)
                if n < 1e-8:
                    print(f'[ERROR] Zero-norm quaternion for {object_name}')
                    return False
                q = q / n
                self.data.qpos[qpos_addr+3:qpos_addr+7] = q
            
            # Get velocity address and validate
            qvel_addr = self.model.jnt_dofadr[joint_id]
            if qvel_addr < 0 or qvel_addr + 6 > len(self.data.qvel):
                print(f'[ERROR] Invalid qvel address for {object_name}: {qvel_addr}')
                return False
            
            # Reset velocities for this object
            self.data.qvel[qvel_addr:qvel_addr+6] = 0.0
            
            # Don't call mj_forward here - let the caller handle it
            # This prevents multiple redundant forward calls
            
            return True
            
        except Exception as e:
            print(f'[ERROR] Failed to set pose for {object_name}: {e}')
            import traceback
            traceback.print_exc()
            return False

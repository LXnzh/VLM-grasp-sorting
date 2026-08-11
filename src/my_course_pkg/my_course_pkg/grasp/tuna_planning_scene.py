import threading

import numpy as np

from my_course_pkg.grasp.tuna_errors import TunaErrorCode, TunaGraspError


TUNA_TABLE_COLLISION_OBJECT_ID = "Desk_1520x900_cell"


def _pose_transform(pose):
    if pose is None:
        return np.eye(4)
    translation = np.array(
        [pose.position.x, pose.position.y, pose.position.z], dtype=float
    )
    x, y, z, w = (
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    )
    quaternion = np.array([x, y, z, w], dtype=float)
    norm = float(np.linalg.norm(quaternion))
    if not np.isfinite(translation).all() or not np.isfinite(norm) or norm <= 0.0:
        raise ValueError("planning-scene table pose is invalid")
    x, y, z, w = quaternion / norm
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def table_top_z_from_planning_scene(
    scene,
    table_object_id=TUNA_TABLE_COLLISION_OBJECT_ID,
):
    world = getattr(scene, "world", None)
    objects = tuple(getattr(world, "collision_objects", ()))
    matches = [item for item in objects if str(getattr(item, "id", "")) == table_object_id]
    if len(matches) != 1:
        raise ValueError(
            "planning scene must contain exactly one exact table collision object"
        )
    collision_object = matches[0]
    frame_id = str(getattr(getattr(collision_object, "header", None), "frame_id", ""))
    if frame_id != "world":
        raise ValueError("planning-scene table collision object must use world frame")
    primitives = tuple(getattr(collision_object, "primitives", ()))
    poses = tuple(getattr(collision_object, "primitive_poses", ()))
    if not primitives or len(primitives) != len(poses):
        raise ValueError("planning-scene table box geometry is missing")
    object_transform = _pose_transform(getattr(collision_object, "pose", None))
    top_values = []
    for primitive, pose in zip(primitives, poses):
        if int(getattr(primitive, "type", -1)) != 1:  # shape_msgs/SolidPrimitive.BOX
            continue
        dimensions = np.asarray(getattr(primitive, "dimensions", ()), dtype=float)
        if dimensions.shape != (3,) or not np.isfinite(dimensions).all() or np.any(
            dimensions <= 0.0
        ):
            raise ValueError("planning-scene table box dimensions are invalid")
        transform = object_transform @ _pose_transform(pose)
        half_extents = 0.5 * dimensions
        world_half_height = float(np.abs(transform[2, :3]) @ half_extents)
        top_values.append(float(transform[2, 3] + world_half_height))
    if not top_values or not np.isfinite(top_values).all():
        raise ValueError("planning-scene table has no usable box top")
    return max(top_values)


def read_tuna_table_z_from_moveit(node, timeout_sec=2.0):
    """Fetch the live MoveIt world geometry; used only by exact-name Tuna."""
    try:
        from moveit_msgs.msg import PlanningSceneComponents
        from moveit_msgs.srv import GetPlanningScene
    except Exception as exc:
        raise TunaGraspError(
            TunaErrorCode.BOUNDS,
            "planning_scene_table",
            {"object_name": "tuna_fish_can", "reason": "moveit_message_import"},
        ) from exc
    client = getattr(node, "_tuna_planning_scene_client", None)
    if client is None:
        client = node.create_client(GetPlanningScene, "/get_planning_scene")
        setattr(node, "_tuna_planning_scene_client", client)
    if not client.wait_for_service(timeout_sec=float(timeout_sec)):
        raise TunaGraspError(
            TunaErrorCode.BOUNDS,
            "planning_scene_table",
            {"object_name": "tuna_fish_can", "reason": "service_unavailable"},
        )
    request = GetPlanningScene.Request()
    request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
    future = client.call_async(request)
    ready = threading.Event()
    future.add_done_callback(lambda _future: ready.set())
    if not ready.wait(float(timeout_sec)):
        raise TunaGraspError(
            TunaErrorCode.BOUNDS,
            "planning_scene_table",
            {"object_name": "tuna_fish_can", "reason": "service_timeout"},
        )
    if future.exception() is not None:
        raise TunaGraspError(
            TunaErrorCode.BOUNDS,
            "planning_scene_table",
            {"object_name": "tuna_fish_can", "reason": "service_exception"},
        ) from future.exception()
    try:
        return table_top_z_from_planning_scene(future.result().scene)
    except (TypeError, ValueError) as exc:
        raise TunaGraspError(
            TunaErrorCode.BOUNDS,
            "planning_scene_table",
            {"object_name": "tuna_fish_can", "reason": str(exc)},
        ) from exc

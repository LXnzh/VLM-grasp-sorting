"""RViz markers and console diagnostics for planned grasps.

The helpers receive the executor rather than owning ROS state.  Keeping that
state on ``ArmMotionExecutor`` preserves the existing node lifecycle and lets
callers continue to replace its debug hooks when needed.
"""

import numpy as np


def gripper_targets(plan):
    """Return the configured open and close positions from a grasp plan."""
    targets = {}
    for step in plan.steps:
        if step.action != "gripper":
            continue
        if step.name == "open_gripper_before_approach":
            targets["open"] = step.gripper_position
        elif step.name == "close_gripper_at_grasp":
            targets["close"] = step.gripper_position
    return targets


def array_text(value, precision=4):
    """Format a numeric array consistently in grasp debug output."""
    return np.array2string(np.asarray(value, dtype=float), precision=precision)


def publish_grasp_debug_markers(
    executor,
    plan,
    *,
    point_type,
    marker_type,
    marker_array_type,
):
    """Publish planned object/TCP geometry for one focused RViz inspection."""
    if not hasattr(executor, "grasp_debug_marker_publisher"):
        return

    debug_info = getattr(plan, "debug_info", {})
    T_world_obj = debug_info.get("T_world_obj")
    if T_world_obj is None:
        return

    final_tcp = np.asarray(plan.grasp_pose_6d, dtype=float)
    pregrasp_tcp = np.asarray(plan.pre_grasp_pose_6d, dtype=float)
    approach_world = np.asarray(
        debug_info.get("approach_direction_world", [0.0, 0.0, 1.0]),
        dtype=float,
    )
    norm = np.linalg.norm(approach_world)
    if norm == 0.0:
        return
    approach_world /= norm

    now = executor.node.get_clock().now().to_msg()
    markers = marker_array_type()

    def sphere(marker_id, position, color):
        marker = marker_type()
        marker.header.frame_id = "world"
        marker.header.stamp = now
        marker.ns = "grasp_debug"
        marker.id = marker_id
        marker.type = marker_type.SPHERE
        marker.action = marker_type.ADD
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.025
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
            *color,
            1.0,
        )
        return marker

    markers.markers.extend(
        [
            sphere(0, T_world_obj[:3, 3], (0.1, 0.9, 0.1)),
            sphere(1, final_tcp[:3], (0.95, 0.15, 0.15)),
            sphere(2, pregrasp_tcp[:3], (0.15, 0.4, 0.95)),
        ]
    )
    arrow = marker_type()
    arrow.header.frame_id = "world"
    arrow.header.stamp = now
    arrow.ns = "grasp_debug"
    arrow.id = 3
    arrow.type = marker_type.ARROW
    arrow.action = marker_type.ADD
    arrow.scale.x = 0.008
    arrow.scale.y = 0.016
    arrow.scale.z = 0.02
    arrow.color.r = 1.0
    arrow.color.g = 0.85
    arrow.color.b = 0.05
    arrow.color.a = 1.0
    arrow_start = final_tcp[:3]
    arrow_end = arrow_start + approach_world * 0.12
    arrow.points = [
        point_type(
            x=float(arrow_start[0]),
            y=float(arrow_start[1]),
            z=float(arrow_start[2]),
        ),
        point_type(
            x=float(arrow_end[0]),
            y=float(arrow_end[1]),
            z=float(arrow_end[2]),
        ),
    ]
    markers.markers.append(arrow)
    executor.grasp_debug_marker_publisher.publish(markers)


def log_grasp_debug(
    executor,
    plan,
    stage,
    *,
    approach_distance,
    pose_formatter,
    transform_to_pose6d,
):
    """Log the selected side-grasp geometry without changing motion policy."""
    debug_info = getattr(plan, "debug_info", {})
    T_world_obj = debug_info.get("T_world_obj")
    T_world_obj_raw = debug_info.get("T_world_obj_raw", T_world_obj)
    final_tcp = np.asarray(plan.grasp_pose_6d, dtype=float)
    pregrasp_tcp = np.asarray(plan.pre_grasp_pose_6d, dtype=float)
    targets = executor._gripper_targets(plan)

    print(f"[GraspDebug] stage={stage}")
    print(f"[GraspDebug] object_name={debug_info.get('object_name', 'unknown')}")
    if T_world_obj is not None:
        if T_world_obj_raw is not None:
            raw_object_pose = transform_to_pose6d(T_world_obj_raw)
            print(
                "[GraspDebug] raw_object_pose_world_base "
                f"{pose_formatter(raw_object_pose)}"
            )
            print(
                "[GraspDebug] raw_object_local_z_axis_world="
                f"{executor._array_text(debug_info.get('raw_object_z_axis_world', []))}"
            )
        object_pose = transform_to_pose6d(T_world_obj)
        final_relative = np.linalg.inv(T_world_obj) @ np.array(
            [
                [1.0, 0.0, 0.0, final_tcp[0]],
                [0.0, 1.0, 0.0, final_tcp[1]],
                [0.0, 0.0, 1.0, final_tcp[2]],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        print(
            "[GraspDebug] canonical_object_pose_world_base "
            f"{pose_formatter(object_pose)}"
        )
        print(
            "[GraspDebug] canonical_object_local_z_axis_world="
            f"{executor._array_text(debug_info.get('canonical_object_z_axis_world', []))}"
        )
        print(
            "[GraspDebug] final_tcp_relative_to_object "
            f"dx_dy_dz={executor._array_text(final_relative[:3, 3])}"
        )
    else:
        print("[GraspDebug] object_pose_world_base unavailable")

    print(
        "[GraspDebug] selected_grasp_library_z="
        f"{debug_info.get('grasp_library_z', float('nan')):.4f} m; "
        "SIDE_GRASP_Z_OFFSET="
        f"{debug_info.get('side_grasp_z_offset', float('nan')):.4f} m"
    )
    print(f"[GraspDebug] final_tcp_target_world_base {pose_formatter(final_tcp)}")
    print(
        "[GraspDebug] pregrasp_tcp_target_world_base "
        f"{pose_formatter(pregrasp_tcp)}"
    )
    print(
        "[GraspDebug] pregrasp_distance_m="
        f"{np.linalg.norm(final_tcp[:3] - pregrasp_tcp[:3]):.4f}; "
        f"configured_approach_offset_m={approach_distance:.4f}"
    )
    print(
        "[GraspDebug] tool_plus_z_approach_axis_world="
        f"{executor._array_text(debug_info.get('approach_direction_world', []))}; "
        "object="
        f"{executor._array_text(debug_info.get('approach_direction_object', []))}"
    )
    print(
        "[GraspDebug] gripper_targets "
        f"open={targets.get('open', float('nan')):.3f}; "
        f"close={targets.get('close', float('nan')):.3f}"
    )

    if stage == "after_final_grasp":
        try:
            actual_tcp = executor.get_current_ee_pose_6d()
            print(
                "[GraspDebug] final_tcp_actual_world_base "
                f"{pose_formatter(actual_tcp)}"
            )
            if T_world_obj is not None:
                actual_relative_xyz = (
                    np.linalg.inv(T_world_obj)
                    @ np.array([*actual_tcp[:3], 1.0], dtype=float)
                )[:3]
                print(
                    "[GraspDebug] final_tcp_actual_relative_to_object "
                    f"dx_dy_dz={executor._array_text(actual_relative_xyz)}"
                )
        except Exception as exc:
            executor.node.get_logger().warn(
                f"Could not read actual final TCP pose for grasp debug: {exc}"
            )

    executor._publish_grasp_debug_markers(plan)

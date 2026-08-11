"""Safe return-to-initial-pose helpers for the arm executor.

All functions operate on the existing executor instance.  This keeps its ROS
subscriptions, action client, and overridable public methods in one place.
"""

import numpy as np


def switch_to_joint_control(executor):
    """Enter joint trajectory control and report whether the switch succeeded."""
    services_ready = (
        executor.arm_api2_client._service_client_list_controllers.service_is_ready()
        and executor.arm_api2_client._service_client_switch_controller.service_is_ready()
    )
    state_ready = (
        executor.arm_api2_client.change_state_to_joint_ctl()
        if services_ready
        else executor.arm_api2_client.change_state_to("JOINT_TRAJ_CTL")
    )
    if not state_ready:
        executor.node.get_logger().warn(
            "change_state_to_joint_ctl failed; retrying direct JOINT_TRAJ_CTL state request."
        )
        state_ready = executor.arm_api2_client.change_state_to("JOINT_TRAJ_CTL")
    if not state_ready:
        executor.node.get_logger().warn(
            "Could not switch arm to JOINT_TRAJ_CTL; return-to-initial may be rejected."
        )
    return state_ready


def send_initial_pose_trajectory(
    executor,
    goal_joint_state,
    *,
    goal_status,
    follow_joint_trajectory,
    joint_trajectory_type,
    joint_trajectory_point_type,
):
    """Use the direct trajectory action as a fallback return-home motion."""
    if not executor.trajectory_action_client.wait_for_server(timeout_sec=2.0):
        executor.node.get_logger().error(
            "Trajectory action server is not available: "
            "/scaled_joint_trajectory_controller/follow_joint_trajectory"
        )
        return False

    traj = joint_trajectory_type()
    traj.joint_names = list(goal_joint_state.name)

    if executor.latest_arm_position_by_name:
        start_point = joint_trajectory_point_type()
        start_point.positions = [
            executor.latest_arm_position_by_name[name]
            for name in traj.joint_names
        ]
        start_point.time_from_start.sec = 0
        traj.points.append(start_point)

    goal_point = joint_trajectory_point_type()
    goal_point.positions = list(goal_joint_state.position)
    goal_point.time_from_start.sec = 4
    traj.points.append(goal_point)

    goal_msg = follow_joint_trajectory.Goal()
    goal_msg.trajectory = traj

    executor.node.get_logger().info(
        "MoveIt return-to-initial failed; sending direct trajectory fallback."
    )
    result = executor.trajectory_action_client.send_goal(goal_msg)
    if result.status == goal_status.STATUS_SUCCEEDED:
        executor.node.get_logger().info("Direct return-to-initial trajectory completed.")
        return True

    executor.node.get_logger().error(
        f"Direct return-to-initial trajectory failed with action status {result.status}."
    )
    return False


def wait_for_arm_joint_state_stable(
    executor,
    *,
    expected_names,
    stable_delta_rad,
    stable_min_duration_sec,
    stable_poll_sec,
    stable_samples_required,
    stable_timeout_sec,
    clock,
):
    """Wait for fresh joint-state samples to become stationary after handoff."""
    observed_sequence = executor._arm_joint_state_sequence
    previous_positions = None
    stable_samples = 0
    stable_since = None
    last_max_delta = None
    deadline = clock.monotonic() + stable_timeout_sec

    while clock.monotonic() < deadline:
        sequence = executor._arm_joint_state_sequence
        positions_by_name = executor.latest_arm_position_by_name
        if (
            sequence > observed_sequence
            and all(name in positions_by_name for name in expected_names)
        ):
            positions = np.asarray(
                [positions_by_name[name] for name in expected_names],
                dtype=float,
            )
            observed_sequence = sequence
            sample_time = clock.monotonic()

            if previous_positions is None:
                stable_samples = 1
                stable_since = sample_time
            else:
                deltas = np.arctan2(
                    np.sin(positions - previous_positions),
                    np.cos(positions - previous_positions),
                )
                last_max_delta = float(np.max(np.abs(deltas)))
                if last_max_delta <= stable_delta_rad:
                    stable_samples += 1
                else:
                    stable_samples = 1
                    stable_since = sample_time

            previous_positions = positions
            stable_duration = sample_time - stable_since
            if (
                stable_samples >= stable_samples_required
                and stable_duration >= stable_min_duration_sec
            ):
                executor.node.get_logger().info(
                    "Arm joint states are stable after control handoff: "
                    f"{stable_samples} fresh samples over {stable_duration:.2f} s, "
                    f"max delta {last_max_delta or 0.0:.5f} rad."
                )
                return True

        clock.sleep(stable_poll_sec)

    last_delta_text = (
        "n/a"
        if last_max_delta is None
        else f"{last_max_delta:.5f} rad"
    )
    detail = (
        "no fresh complete /joint_states sample"
        if previous_positions is None
        else (
            f"only {stable_samples}/{stable_samples_required} "
            f"stable fresh samples (last max delta={last_delta_text}, "
            f"required stable duration={stable_min_duration_sec:.2f} s)"
        )
    )
    executor.node.get_logger().warn(
        "Arm did not become stationary after switching to JOINT_TRAJ_CTL; "
        f"not sending a return-to-initial trajectory ({detail})."
    )
    return False


def return_to_initial_pose(
    executor,
    *,
    verify_init_pose_node,
    clock,
):
    """Return the arm home without bypassing control-mode safety gates."""
    deadline = clock.monotonic() + 5.0
    while not executor.latest_arm_joint_names and clock.monotonic() < deadline:
        clock.sleep(0.1)

    if not executor.latest_arm_joint_names:
        executor.node.get_logger().warn(
            "No arm joint names received from /joint_states; skipping return to initial pose."
        )
        return False

    print("Returning to initial joint pose...")
    print(
        "Initial pose joint names "
        f"({len(executor.latest_arm_joint_names)}): {executor.latest_arm_joint_names}"
    )
    if not executor.switch_to_joint_control():
        executor.node.get_logger().warn(
            "Could not switch to JOINT_TRAJ_CTL; skipping return-to-initial "
            "trajectory rather than issuing it in the wrong control mode."
        )
        return False
    if not executor.wait_for_arm_joint_state_stable():
        return False

    goal_joint_state = verify_init_pose_node.build_initial_joint_goal(
        executor.latest_arm_joint_names,
        executor.node.get_clock().now().to_msg(),
    )

    for attempt in range(1, 3):
        print(f"Return-to-initial move_to_joint attempt {attempt}/2")
        try:
            if executor.arm_api2_client.move_to_joint(goal_joint_state):
                return True
        except Exception as exc:
            executor.node.get_logger().warn(
                f"Exception while returning to initial joint pose: {exc}"
            )
        clock.sleep(0.5)

    if executor.send_initial_pose_trajectory(goal_joint_state):
        return True

    executor.node.get_logger().warn(
        "Failed to return to initial joint pose. Pick/place sequence finished, "
        "but the final reset move was not accepted or did not complete."
    )
    return False

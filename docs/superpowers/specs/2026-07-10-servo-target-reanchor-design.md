# Servo Target Reanchor Design

## Goal

Make `SERVO_POS_CTL` converge to repeated absolute pose commands without
restarting motion from stale commanded joint targets.

## Evidence

The final TCP feedback guard correctly commanded the same target at 50 ms
intervals. It reduced the Cartesian error from 30.5 mm to 10.0 mm, then
drifted to a 16.0 mm timeout. The FoundationPose target and its frame were
verified correct.

In `m2Iface::move_to_pose_servo`, a new target received while an interpolation
is active uses `active_target_positions_` as the next interpolation start. This
is the prior commanded target, not the current joint state. The servo loop also
clears active interpolation after `t >= 1.4`, even when identical pose commands
continue to arrive.

## Scope

- Change only `src/arm_api2/src/moveit2_iface.cpp` servo target handling.
- Keep existing IK, collision checks, velocity and acceleration smoothing, and
  the five-second no-target watchdog.
- Do not change FoundationPose, grasp selection, trajectory planning, or the
  final TCP safety guard.

## Design

### Target Callback

After IK produces a joint target:

1. Compare it with the active joint target using a per-joint absolute epsilon
   of `1e-4` radians.
2. Log the maximum joint delta with a throttled info message so live tuning can
   distinguish identical commands from meaningful target changes.
3. For an identical target, update `last_target_time_` and return without
   resetting the interpolation start, duration, or smoother.
4. For a changed target, obtain current joint positions from
   `m_moveGroupPtr->getCurrentState(1.0)` and use them as
   `active_start_positions_`.
5. If current state or its joint model group is unavailable, log a warning and
   return without modifying the active target or interpolation state.
6. Set the new active target, start time, and duration from the target command
   interval.

The implementation must not read `q_out_` from the target callback because the
servo timer updates it independently. Using the MoveIt current state avoids a
new synchronization dependency.

### Servo Loop

Remove the `t >= 1.4` block that clears active interpolation. Once normalized
time reaches one, the existing smoother can continue driving `q_out_` to the
unchanged target. The existing five-second no-target watchdog remains the only
mechanism that clears an idle target.

## Expected Behavior

- Repeated identical `send_pose_cmd` calls refresh the watchdog but do not
  restart interpolation.
- A changed target begins from real current joints rather than a stale command.
- Failure to obtain current state is safe: the last known active trajectory is
  left intact.
- The final TCP feedback guard should then converge inside 5 mm or safely
  timeout without drift caused by target restarts.

## Verification

1. Build `arm_api2` and restart the MuJoCo/MoveIt launch so the C++ node reloads.
2. Reset, run a fresh pipeline, then run one debug-stopped grasp demo.
3. Confirm the throttled log reports near-zero maximum joint delta for repeated
   final-pose commands.
4. Confirm `Final approach converged` reports position error at or below 5 mm.
5. Confirm a timeout still prevents the gripper from closing.

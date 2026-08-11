# Final Approach Convergence Design

## Goal

Ensure the robot reaches the selected final TCP before closing the gripper.
This change addresses the tomato-can side grasp where FoundationPose and grasp
selection were correct, but the servo controller stopped short of the final
TCP.

## Evidence

The reset, pipeline, and single grasp-demo run on 2026-07-10 produced:

- FoundationPose tomato pose within about 1 mm of the simulation truth.
- A successful MoveIt pregrasp for candidate 3.
- Final TCP target `[-0.0539, -0.9248, 0.9799]`.
- Final TCP actual `[-0.0320, -0.9181, 0.9910]`.
- Position error about 25.5 mm, including about 23.5 mm short along the
  configured approach direction.

The current Cartesian path publishes timed pose commands but does not confirm
that the controller has reached the final pose. Consecutive interpolation
segments use the prior command target as their next start point, so controller
lag can accumulate unnoticed.

## Options Considered

1. Add final-pose feedback convergence after the Cartesian approach.
   This is the selected option. It preserves the current straight approach and
   adds the missing safety check at the point where accuracy matters.
2. Rebuild every interpolation segment from the latest actual TCP pose.
   This reduces accumulated lag but does not guarantee final convergence and
   changes shared interpolation behavior.
3. Execute the final approach through MoveIt.
   This has action feedback, but it may not preserve the desired straight
   approach and adds planning failures near the object.

## Scope

- Apply the convergence check only after a plan step reaches its final grasp
  TCP. Pregrasp MoveIt motions, object pose estimation, grasp selection,
  gripper commands, lift, transfer, and drop remain unchanged.
- Treat convergence failure as terminal for that plan execution. Do not close
  the gripper and do not try another geometric candidate, because the observed
  failure is controller tracking rather than candidate geometry.
- Keep debug-stop behavior, but enter it only after the final TCP has either
  converged or raised an explicit failure.

## Design

### Configuration

Add environment-backed settings in `grasp/config.py`:

- `GRASP_FINAL_APPROACH_POSITION_TOLERANCE_M`, default `0.005`.
- `GRASP_FINAL_APPROACH_SETTLE_TIMEOUT_SEC`, default `1.5`.
- `GRASP_FINAL_APPROACH_COMMAND_PERIOD_SEC`, default `0.05`.

### Executor Behavior

After the existing Cartesian final-approach motion completes, the executor
will run a final-pose convergence loop:

1. Construct the existing final TCP pose in the `world` frame.
2. Read the actual TCP from `/arm/state/current_pose`.
3. Compute Euclidean position error to the final TCP.
4. If the error is at most the configured tolerance, continue to the existing
   debug or gripper-close flow.
5. Otherwise, publish the exact final TCP pose command again, wait one command
   period, and repeat until the timeout expires.
6. On timeout, raise a dedicated convergence error containing the target pose,
   actual pose, and final position error. The exception propagates past
   candidate selection; no close command is sent.

The loop publishes the fixed final pose directly instead of calling the
interpolator again. The interpolator intentionally reuses its latest commanded
endpoint for nearby calls, which is precisely the assumption that hid the
controller lag.

### Diagnostics

Each convergence attempt will log the current position error. Success logs the
number of command cycles and final error. Timeout logs the target and actual
TCP values. The existing `[GraspDebug] final_tcp_actual_world_base` output
remains the post-convergence measurement.

## Tests

Extend executor tests with lightweight fake arm clients:

- A pose sequence that converges within tolerance publishes the final target
  until success and then permits the next step.
- A pose sequence that stays outside tolerance raises the convergence error,
  does not send a gripper close command, and does not advance to another plan.
- Existing debug-stop tests continue to stop after a successful convergence.

Run the focused executor test module and the existing planner and trajectory
tests. Then build `my_course_pkg` with `colcon`.

## Live Verification

1. Reset the simulation and run `pipeline`.
2. Run one `grasp_demo` with `PYTHONUNBUFFERED=1` and
   `GRASP_DEBUG_STOP_AT_GRASP=1`.
3. Confirm the canonical tomato pose remains near
   `[-0.1355, -0.9330, 0.8900]`.
4. Confirm the final TCP position error is at most 5 mm before the debug stop.
5. Run a normal trial and confirm the gripper closes only after convergence.

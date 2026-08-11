# Vertical Pregrasp Stability Design

## Goal

Prevent the per-run vertical command-offset calibration from freezing while
the arm is still moving. Preserve the existing 1.5 mm world-XY and 3 mm Z
physical gates, the 30 mm offset cap, the three-command calibration limit, and
all nominal descent-waypoint checks.

This is a narrow follow-up to
`2026-07-14-vertical-command-offset-calibration-design.md`. It supersedes that
document's success rule that median feedback alone is sufficient. A successful
pregrasp now requires a stationary three-sample window whose median and newest
sample both pass the existing physical gates.

## Live Evidence

The first live calibrated foam-brick run learned a bounded offset in two
commands:

```text
[-0.2, -8.9, +21.4] mm
```

The final three-sample window reported a median approximately 1.0 mm from the
nominal target in XY and 1.7 mm in Z, so the implementation froze the offset.
However, the newest raw sample was already approximately 1.7 mm away in XY and
3.7 mm in Z and was still moving. The window's Z peak-to-peak change was about
2.0 mm.

After calibration returned, `move_vertical_approach()` redundantly requested
`SERVO_POS_CTL` even though `execute_plan()` had already entered that mode
before calibration. The first descent precheck then observed 2.2 mm lateral
error and held before issuing a lower waypoint. The stop was safe, but the
calibration had declared convergence before feedback was stationary.

The screenshot looked visually centered, but the prior estimated nominal
per-finger foam-brick clearance is only about 2.3 mm. Widening the descent gate
would consume that physical margin and is rejected.

## Considered Approaches

### Three-sample geometric spread

Measure the maximum pairwise XY distance and Z peak-to-peak change within each
existing three-sample window. Require the window to be stationary before
evaluating position convergence. This is direction-independent, detects
one-way drift and oscillation, and reuses the current feedback samples. This is
the selected approach.

### Two consecutive accepted windows

Require two complete median/latest windows to pass. This avoids a separate
spread threshold but adds fixed latency and can still hide motion inside either
window. It is not selected.

### Fitted Cartesian velocity

Fit XYZ velocity from sample timestamps and gate on estimated speed. This is
more complex, sensitive to timing jitter, and unnecessary for a three-sample
window. It is not selected.

## Stationary Window Definition

Continue collecting `GRASP_VERTICAL_REANCHOR_SAMPLE_COUNT` samples, default
three, separated by `GRASP_FINAL_APPROACH_COMMAND_PERIOD_SEC`, currently
0.05 seconds. The approved default therefore spans 0.1 seconds. A valid odd
sample-count override intentionally expands the window; the spread limits do
not scale with window duration.

For the sample translations `sample_xyz`, define:

```text
xy_spread_m = max(
    norm(sample_xyz[i, :2] - sample_xyz[j, :2])
    for every i < j
)

z_spread_m = max(sample_xyz[:, 2]) - min(sample_xyz[:, 2])
```

The XY definition is the maximum Euclidean distance between any two sampled XY
points. It is not a separate X range and Y range, and it is not only the
first-to-last displacement.

Derive stationary limits from the existing physical gates:

```text
stationary_xy_limit_m = 0.5 * VERTICAL_APPROACH_XY_TOLERANCE_M
stationary_z_limit_m = 0.5 * VERTICAL_APPROACH_Z_TOLERANCE_M
```

With current defaults, these are 0.75 mm XY and 1.5 mm Z. A window is
stationary when both spreads are finite and less than or equal to their limits.
Equality passes. No new environment configuration is added.

## Calibration Decision Order

For every feedback window, apply this order:

1. Collect the full sample window, retain the newest raw full 6D pose, compute
   component-wise median XYZ, and compute XY/Z spread.
2. Check stationarity before position convergence, residual, divergence,
   offset, or command-count decisions.
3. If the window is not stationary, send no compensated command, do not update
   the offset, and do not increment or exhaust the compensated-command count.
   Wait one existing command period and collect another complete window.
4. If the window is stationary, clear any active stationary-wait deadline and
   independently evaluate both:
   - median XYZ against the nominal pregrasp using the existing 1.5 mm XY and
     3 mm Z gates;
   - newest raw XYZ against the same nominal pregrasp and gates.
5. Freeze and return the current offset only when the stationary window's
   median and newest sample both pass.
6. If the window is stationary but either position check fails, calculate the
   residual from the median XYZ and continue the existing bounded cumulative
   compensation algorithm.

Residual divergence, the 30 mm cumulative offset cap, and the three-command
limit are evaluated only from stationary windows. This prevents a transient
moving sample from being mistaken for a steady-state mapping error.

## Bounded Stationary Wait

Classifying the first non-stationary window starts a stationary-wait deadline
using the existing `FINAL_APPROACH_SETTLE_TIMEOUT_SEC`, currently 1.5 seconds.
The already-completed first window does not count retroactively. Every
subsequent inter-window wait and complete sample window counts toward the
deadline.

Subsequent non-stationary windows do not restart or extend that deadline. If a
stationary window arrives before the deadline, clear it and proceed with the
position decision. Every newly sent compensated command also begins the next
feedback phase with no inherited stationary deadline.

If feedback remains non-stationary at the deadline:

- command a hold at the newest valid raw full 6D pose;
- raise `VerticalApproachConvergenceError` at stage `pregrasp`;
- report that feedback did not become stationary;
- preserve the current compensated-command count;
- issue no descent or gripper-close command.

## Servo-Mode Continuity

`execute_plan()` remains responsible for entering `SERVO_POS_CTL` immediately
after the MoveIt pregrasp step and before starting calibration. A successful
calibration proves that the executor is already in the required mode.

Remove the redundant `enter_servo_pos_mode()` call from
`move_vertical_approach()`. The accepted calibration state then flows directly
into the first descent precheck without another controller-state request or its
associated delay. The sole production caller is `execute_plan()`; the method's
docstring and tests will make this precondition explicit.

## Component Changes

### `executor.py`

- Extend pregrasp sampling to return the complete XYZ sample matrix in addition
  to median XYZ and newest full pose.
- Add a focused helper that computes XY and Z spread and the derived limits.
- Apply stationary-wait state and median/latest convergence checks inside
  `converge_at_vertical_pregrasp()`.
- Remove the redundant mode request from `move_vertical_approach()`.
- Leave descent commands, frozen-offset handling, nominal waypoint feedback,
  final verification, and non-vertical paths unchanged.

### `config.py`

No change. Stationary limits derive from the existing vertical tolerances, and
the wait reuses the existing settle timeout.

### Planner and selector

No change. Grasp selection, bounds centering, nominal poses, interpolation,
orientation, gripper commands, lift, return, and release remain unchanged.

## Diagnostics

Every pregrasp sample-window log includes:

- all sampled XYZ translations;
- median XYZ;
- newest XYZ;
- XY maximum pairwise spread and its derived limit;
- Z peak-to-peak spread and its derived limit;
- whether the window is stationary.

A non-stationary retry logs that no compensated command was issued and includes
the remaining wait time. Success continues to log the frozen offset and command
count. A timeout log distinguishes stationary-wait failure from offset-cap,
divergence, and command-exhaustion failures.

## Tests

Executor tests will prove:

- the recorded final live window does not freeze because its newest sample and
  Z spread fail even though its median passes;
- a non-stationary window sends no compensated command, does not change the
  offset, and does not increment the compensated-command count;
- a later stationary window resumes normal position evaluation;
- stationary median and newest feedback must both pass before freezing;
- a stationary window with a failed position check uses median residual for the
  next bounded cumulative correction;
- exactly 0.75 mm XY spread and 1.5 mm Z spread pass with current defaults;
- XY uses maximum pairwise Euclidean distance and catches diagonal or
  first-to-last-cancelling motion;
- repeated non-stationary windows exceeding 1.5 seconds hold the newest full
  pose and fail before descent or close;
- a newly issued compensation resets the stationary-wait phase;
- `move_vertical_approach()` does not request servo mode again;
- one vertical plan still enters servo mode exactly once before calibration and
  passes the unchanged frozen offset into descent;
- existing cap, divergence, exhaustion, nominal waypoint, final verification,
  return-motion, and non-vertical behavior remain unchanged.

Run the focused executor/config tests, the related selector/planner/trajectory/
executor/evaluation regression suite, Python compilation, and a
`my_course_pkg` symlink build in the Docker-mounted workspace. Confirm modified
runtime/test files match between both host workspaces apart from line endings.

## Live Validation

After rebuilding, restart the full simulator launch, verify unique arm and
gripper servers, reset, and run fresh foam-brick perception. Use
`GRASP_DEBUG_STOP_AFTER_CLOSE=1` for the first bounded trial.

A qualifying trial must show:

- any moving feedback window is logged as non-stationary without a new command;
- stationarity, median feedback, and newest feedback all pass before the offset
  is frozen;
- there is no second `SERVO_POS_CTL` request between frozen calibration and the
  first descent waypoint;
- the first and later descent waypoints continue to pass against nominal poses;
- neither finger contacts the brick's top surface before close;
- the brick is visibly between both fingers at the after-close stop.

Only after that bounded close trial passes should bounded lift validation
resume.

## Acceptance Criteria

The work is complete when:

- the stationary definition and 1.5-second wait are implemented exactly as
  specified;
- median and newest feedback must both pass before calibration freezes;
- moving windows never update compensation or consume a command iteration;
- calibration flows into descent without a redundant mode request;
- the 1.5 mm XY, 3 mm Z, 5 mm waypoint, 30 mm offset, three-command, and
  divergence rules remain unchanged;
- focused tests, related regression, compilation, and package build pass;
- the bounded live close-only trial enters descent and captures the brick
  without top-surface contact.

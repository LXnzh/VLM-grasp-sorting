# Vertical Descent XY Tolerance Design

## Goal

Allow a vertical grasp descent to continue when its measured world-XY tracking
error is at most `2.0 mm`, while preserving the existing stricter `1.5 mm`
pregrasp calibration gate. Keep all Z, timeout, hold, and fail-closed behavior
unchanged.

## Observed Failure

Fresh trials for `gelatin_box` and `sponge` repeatedly reached a visually
centered, top-down pose with both fingertips beside the object, then stopped
before `close_gripper_at_grasp`:

- gelatin trial 5 stopped at waypoint 20 of 21 with approximately `1.5 mm` XY
  error and `0.3 mm` Z error;
- sponge trial 6 stopped at waypoint 18 of 20 with `1.6 mm` XY error and
  `0.2 mm` Z error.

Both residuals were almost entirely in world `+Y`. Both satisfied the existing
`3.0 mm` Z gate and exceeded the `1.5 mm` XY gate only at its boundary. The
gripper close command was never sent, so these runs do not demonstrate weak
closing or an invalid object pose. They demonstrate that one XY tolerance is
currently serving two different safety purposes: precise calibration at safe
hover and practical tracking during the already-gated descent.

## Selected Design

Keep `GRASP_VERTICAL_APPROACH_XY_TOLERANCE_M` at its existing `0.0015` default
and continue using it for pregrasp calibration, median/latest convergence, and
pregrasp stationarity limits.

Add a separately validated configuration value:

```text
GRASP_VERTICAL_DESCENT_XY_TOLERANCE_M=0.0020
```

Use the new descent tolerance for all XY decisions after strict pregrasp
calibration has passed and its command offset has been frozen:

- lateral settling before sending a lower vertical waypoint;
- convergence after each commanded vertical waypoint;
- final vertical grasp verification immediately before gripper close.

The same `2.0 mm` value must be used consistently across these three checks.
Accepting a waypoint under one tolerance and then checking it under a stricter
tolerance before the next waypoint would only move the failure to the next
stage.

## Error And Logging Semantics

Pass the applicable XY tolerance explicitly into the shared vertical success
logger and hold-and-raise path. `VerticalApproachConvergenceError` must record
and print the tolerance used by the stage that actually failed:

- pregrasp errors report `0.0015 m` by default;
- descent and final-verification errors report `0.0020 m` by default.

Do not infer the tolerance from a stage-name string. Explicit data flow keeps
the safety decision, log, and exception consistent and makes tests independent
of naming conventions.

## Safety Boundaries

- Keep the vertical Z tolerance at `3.0 mm` for pregrasp, every waypoint, and
  final verification.
- Keep the existing `1.5 s` convergence timeout and command sampling period.
- An XY error greater than `2.0 mm` during descent still waits without sending
  a lower waypoint and then holds the observed pose on timeout.
- A post-command waypoint or final verification above `2.0 mm` still holds and
  raises before close.
- Keep pregrasp calibration at `1.5 mm`; do not broaden its stationarity window
  as a side effect of the descent change.
- Do not change command-offset calibration, the 30 mm offset cap, waypoint
  spacing, grasp target height, gripper opening, close position, close effort,
  or lift-return behavior.
- Do not add `gelatin_box`, `sponge`, or any other object-specific branch.
- Side, centered, and round-top profile tolerances remain unchanged.

## Verification

Add or update deterministic tests to prove:

- the new descent XY tolerance defaults to `0.0020` and rejects non-finite or
  non-positive configuration values;
- pregrasp calibration still rejects or corrects an error above `0.0015`;
- descent pre-command settling accepts an error between `1.5 mm` and `2.0 mm`;
- post-command waypoint convergence accepts an error between `1.5 mm` and
  `2.0 mm`;
- final vertical verification accepts an error between `1.5 mm` and `2.0 mm`;
- each descent-stage check still holds and fails above `2.0 mm`;
- Z error above `3.0 mm` still fails even when XY is within `2.0 mm`;
- success logs and raised errors report the stage's actual XY tolerance.

Run the focused configuration, executor, trajectory-planner, and grasp
regression tests, Python compilation, fatal flake8 for changed Python files,
and a Docker `colcon build --symlink-install` for `my_course_pkg`.

For live validation, restart the experiment launch so the new default is
loaded and set `GRASP_DEBUG_STOP_AFTER_CLOSE=1` before launch. Run fresh
gelatin-box and sponge trials separately. A qualifying run must log the strict
pregrasp `0.0015 m` gate, descent/final `0.0020 m` gates, execute
`close_gripper_at_grasp`, and stop after close with the object visibly between
the fingertips. Do not proceed directly to a full lift-return trial.

## Non-Goals

- Fixing the separate cached-feedback freshness issue observed during one
  pregrasp calibration trial.
- Proving that the nominal gripper opening is sufficient for every box pose.
- Changing grasp selection, perception, FoundationPose, scene placement, or
  collision-clearance policy.
- Automatically performing simulator or robot motion during implementation.

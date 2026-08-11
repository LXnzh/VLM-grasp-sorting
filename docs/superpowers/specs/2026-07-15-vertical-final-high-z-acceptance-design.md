# Vertical Final High-Z Acceptance Design

## Goal

Allow a vertical grasp to close when the robot has settled slightly above the
nominal final grasp Z but is still laterally aligned and physically inside the
live target height bounds. Keep the existing strict behavior for pregrasp,
intermediate descent waypoints, downward/table-side Z errors, and missing or
invalid geometry.

## Observed Failure

The fresh `gelatin_box` trial reached all 21 vertical descent waypoints and
visually placed both fingertips beside the box, but stopped before
`close_gripper_at_grasp`:

- final target XYZ: `[-0.6016, -0.7516, 0.8980]`;
- final measured XYZ: `[-0.6013, -0.7504, 0.9026]`;
- XY error: approximately `1.3 mm`, which passes the controlled `2.0 mm`
  vertical-descent XY tolerance;
- signed Z residual (`actual_z - target_z`): approximately `+4.6 mm`, which
  exceeds the current symmetric `3.0 mm` Z tolerance;
- live object bounds: bottom approximately `0.8900 m`, center approximately
  `0.9053 m`, and top approximately `0.9206 m`.

The actual TCP Z was still inside the object's live height range and below its
center. The positive residual means that the robot remained above the nominal
target rather than travelling farther toward the table. No gripper close
result was logged, so this failure was caused by the final Z gate and does not
show that the gripper failed to clamp the object.

## Considered Approaches

### Increase the global symmetric Z tolerance to 5 mm

This is the smallest code change, but it would also relax pregrasp,
intermediate waypoints, and negative Z errors toward the table. That broadens
the safety envelope far beyond the observed failure and is rejected.

### Raise the planned vertical grasp target

This could reduce the measured residual for the current object, but it changes
the intended grasp geometry for every affected vertical object. It also hides
the distinction between a safe, bounded high-side tracking residual and a
different desired grasp pose. This may be reconsidered later using broader
object-level evidence, but it is not selected for this fix.

### Add final-only asymmetric, bounds-gated acceptance

This directly represents the observed safe condition: preserve the existing
`3.0 mm` gate everywhere, then permit up to `5.0 mm` only on the high side at
the final grasp and only while the actual Z remains inside valid live target
bounds. This is the selected approach.

## Selected Acceptance Rule

Add a separately validated configuration value with this default:

```text
GRASP_VERTICAL_FINAL_HIGH_Z_TOLERANCE_M=0.0050
```

It must be finite, positive, and no smaller than the existing
`GRASP_VERTICAL_APPROACH_Z_TOLERANCE_M`, whose default remains `0.0030`.

Define the signed Z residual as:

```text
signed_z_error_m = actual_z - target_z
```

Pregrasp and every non-final vertical waypoint retain the existing symmetric
rule:

```text
abs(signed_z_error_m) <= 0.0030
```

At the last vertical waypoint and the close-readiness verification, Z passes
when either the ordinary rule passes or every controlled-high condition passes:

```text
0.0030 < signed_z_error_m <= 0.0050
- bounds_bottom_z <= target_z <= bounds_top_z
- bounds_bottom_z <= actual_z <= bounds_top_z
```

The first controlled-high inequality is one-sided: an actual pose below the
target by more than `3.0 mm` still fails. The existing final XY requirement of
at most `2.0 mm` is independent and must also pass.

For the commanded final waypoint, ordinary convergence is checked throughout
the existing settle window. The controlled-high fallback is evaluated at the
end of that window, rather than accepting the first high-side sample while the
arm may still be moving. A qualifying fallback returns success without sending
another lower command. The close-readiness check immediately following the
settled waypoint uses the same acceptance predicate, so the waypoint cannot
pass under one Z policy and fail under another.

## Live Bounds Data Flow

Use the vertical target bounds already computed from the selected live scene
object. Preserve bottom and center in plan debug data and make the top explicit,
or derive it equivalently from the same values:

```text
bounds_top_z = 2 * bounds_center_z - bounds_bottom_z
```

The executor validates that bottom, center, and top are finite and ordered:

```text
bounds_bottom_z < bounds_center_z < bounds_top_z
```

It extracts the bounds once from the vertical plan and passes the resulting
final-Z context to both the last-waypoint gate and final verification. The
high-side fallback is disabled when the plan is not vertical, a required value
is absent or non-finite, ordering is invalid, or either target or actual Z lies
outside the bounds. Disabled fallback means the original symmetric `3.0 mm`
rule remains authoritative and the existing hold-and-raise path is preserved.

Do not select the policy by object name. `gelatin_box`, `sponge`, and any other
vertical object receive identical treatment when their live geometry satisfies
the same conditions.

## Error And Logging Semantics

Refactor the vertical Z decision into one explicit helper used by the final
waypoint and close-readiness gates. It returns enough structured information to
keep the decision and diagnostics consistent, including:

- signed and absolute Z error;
- ordinary low/high tolerance and configured final high-side tolerance;
- bounds bottom, center, and top when valid;
- whether controlled-high acceptance was eligible and used;
- a specific rejection reason such as missing bounds, invalid bounds, target
  outside bounds, actual above bounds, or residual above `5.0 mm`.

Success logs distinguish ordinary convergence from controlled-high acceptance.
The latter must clearly state that no additional lower command was sent.
`VerticalApproachConvergenceError` and the preceding hold log report the signed
residual and the applicable asymmetric limits instead of presenting the final
gate as a symmetric `5.0 mm` tolerance.

## Safety Boundaries

- Keep pregrasp XY at `1.5 mm` and descent/final XY at `2.0 mm`.
- Keep Z at symmetric `3.0 mm` for pregrasp and every non-final waypoint.
- Keep the negative/downward final Z limit at `3.0 mm`.
- Permit the `5.0 mm` value only for positive/high-side final residuals.
- Require valid live bounds and require both target and actual Z inside them.
- Evaluate the last-waypoint fallback only after the existing settling window.
- Do not issue another lower correction when controlled acceptance is used.
- Keep timeout duration, waypoint spacing, calibrated command offset, maximum
  offset, gripper opening, close position, close effort, and debug-stop behavior
  unchanged.
- Preserve hold-and-raise behavior for every rejected case.
- Do not add an object-specific exception.

## Verification

Add deterministic tests that prove:

- the new high-side tolerance defaults to `0.0050` and rejects non-finite,
  non-positive, or below-base values;
- pregrasp and non-final waypoints still fail at positive or negative
  `4.6 mm` Z error;
- a final `+4.6 mm` residual passes only after the ordinary settle window when
  XY is within `2.0 mm` and target/actual Z are inside valid bounds;
- the same final pose passes close-readiness verification;
- a final `-4.6 mm` residual fails;
- residuals greater than `+5.0 mm` fail;
- actual Z above top or below bottom fails;
- target Z outside the live bounds fails;
- missing, non-finite, reversed, or zero-height bounds disable the fallback and
  preserve the `3.0 mm` failure;
- non-vertical profiles never use this fallback;
- ordinary final convergence within `3.0 mm` still succeeds without requiring
  bounds;
- success and failure diagnostics report signed residuals, bounds, and whether
  controlled-high acceptance was used;
- the execution sequence sends `close_gripper_at_grasp` only after both the
  final waypoint and close-readiness gates pass.

Run the focused configuration, executor, trajectory-planner, and grasp
regression suites; Python compilation; fatal flake8 on changed Python files;
and a Docker `colcon build --symlink-install` for `my_course_pkg`. Confirm the
installed runtime reports defaults of `1.5 mm` pregrasp XY, `2.0 mm` descent
XY, `3.0 mm` ordinary Z, and `5.0 mm` final high-side Z.

For live validation, restart the experiment launch so the new configuration is
loaded and keep `GRASP_DEBUG_STOP_AFTER_CLOSE=1`. Run a fresh `gelatin_box`
trial first. A qualifying controlled acceptance must log valid bounds, an XY
error no greater than `2.0 mm`, a positive Z residual no greater than `5.0 mm`,
`close_gripper_at_grasp`, and the debug stop immediately after close with the
object visibly between the fingertips. Do not proceed directly to lift-return;
repeat the close-only check for `sponge` after gelatin is confirmed.

## Non-Goals

- Relaxing the downward/table-side final Z boundary.
- Changing the planned vertical grasp height based on this single residual.
- Fixing the separate cached-feedback freshness issue.
- Tuning gripper width or effort.
- Changing perception, FoundationPose, object spawning, scene selection, or
  collision-clearance policy.
- Automatically launching simulator or robot motion during implementation.

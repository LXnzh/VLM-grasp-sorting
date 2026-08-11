# Vertical Final Center-Bounded Acceptance Design

## Status And Scope

This design supersedes the fixed `5.0 mm` positive-Z cap in
`2026-07-15-vertical-final-high-z-acceptance-design.md`. It changes only the
final settled Z acceptance rule for the `vertical` grasp profile. It does not
change planned poses, motion commands, gripper commands, object geometry, or
the ordinary convergence rule.

The affected profile currently contains exactly these objects:

- `pudding_box`;
- `gelatin_box`;
- `sponge`;
- `foam_brick`;
- `rubiks_cube`.

The other 13 experiment objects use `side`, `round_top`, `centered`, or
`top_down` profiles and must not enter this policy.

## Goal

Allow a vertical grasp to close when its final settled TCP remains slightly
above the nominal target but is laterally aligned and no higher than the live
object center. Preserve every execution accepted by the unchanged ordinary
`3.0 mm` rule, especially the already stable `sponge`, `foam_brick`, and
`rubiks_cube` grasps.

## Evidence

The latest `gelatin_box` trial reached its final waypoint with:

- final target Z approximately `0.8980 m`;
- final actual Z approximately `0.9032 m`;
- signed Z residual approximately `+5.2 mm`;
- XY error approximately `0.6 mm`;
- live bottom, center, and top Z approximately `0.8900 m`, `0.9053 m`, and
  `0.9205 m`.

Both fingertips visually straddled the box, but the fixed `5.0 mm` gate rejected
the pose before the close command. The actual TCP was about `2.1 mm` below the
live object center. For that run, the live center implied an effective positive
limit of about `7.3 mm` relative to the target.

## Considered Approaches

### Increase the fixed cap

Raising the cap to `6-8 mm` would pass the observed trial, but the value would
remain arbitrary and would not adapt to different object heights or live poses.

### Use the live object center as the upper boundary

Retain ordinary symmetric acceptance through `3.0 mm`. For a settled final
positive residual beyond `3.0 mm`, accept only when the target is inside valid
live object bounds and the actual TCP lies between the live bottom and center.
This is selected because it describes the physical condition directly and
adapts independently to each object instance.

### Use the live object top as the upper boundary

The top provides the widest geometry-based interval, but it could accept a TCP
that is too high for reliable finger overlap. This is rejected.

## Acceptance Rule

Define:

```text
signed_z_error_m = actual_z - target_z
ordinary_z_tolerance_m = 0.0030
```

The existing ordinary rule is evaluated first:

```text
abs(signed_z_error_m) <= ordinary_z_tolerance_m
```

Ordinary success requires no live bounds. This ordering is a non-regression
requirement: every final pose accepted before this change remains accepted even
if bounds are missing or invalid.

When the ordinary rule fails, the controlled-center fallback passes only if all
of these conditions hold:

```text
profile == "vertical"
stage == settled final waypoint or immediate close-readiness verification
signed_z_error_m > ordinary_z_tolerance_m
bottom_z < center_z < top_z, with every value finite
bottom_z <= target_z <= top_z
bottom_z <= actual_z <= center_z
final_xy_error_m <= 0.0020
```

The center boundary is inclusive. The fallback has no independent fixed
positive-Z cap; its effective per-run limit is:

```text
effective_positive_limit_m = center_z - target_z
```

If this effective limit is no greater than `3.0 mm`, the fallback adds no
acceptance range. A negative residual beyond `3.0 mm` remains rejected.

## Execution And Data Flow

The executor continues to derive bottom, center, and top from the selected
vertical plan's live debug geometry. It validates the bounds once and supplies
the same immutable bounds context to both the final waypoint gate and the
close-readiness gate.

The final waypoint first runs the existing command and settle sequence. Only
after the settle window ends may the controlled-center fallback be evaluated.
A controlled-center success returns without sending another lower command. The
immediately following close-readiness check uses the same decision helper and
therefore cannot disagree about the Z policy.

The fixed `GRASP_VERTICAL_FINAL_HIGH_Z_TOLERANCE_M` value must no longer decide
acceptance. Remove the setting and its validation instead of leaving a stale
configuration knob that appears authoritative.

## Compatibility And Safety

- Keep pregrasp XY at `1.5 mm`.
- Keep descent and final XY at `2.0 mm`.
- Keep symmetric Z at `3.0 mm` for pregrasp and every non-final waypoint.
- Keep the final negative/downward Z limit at `3.0 mm`.
- Do not require bounds for ordinary final success within `3.0 mm`.
- Do not alter targets, waypoint spacing, reanchoring, offsets, timeouts,
  gripper opening, close position, effort, or debug-stop behavior.
- Preserve existing hold-and-raise handling for rejected poses.
- Do not add object-name exceptions.

Consequently, stable `sponge`, `foam_brick`, and `rubiks_cube` runs on the
ordinary path are behaviorally identical. The only new behavior for those
objects is that a future settled final pose in the controlled interval may
proceed where it previously stopped. A `pudding_box` failure that occurs during
planning or IK remains unaffected because this policy is reached only during
final execution.

Compared with the experimental fixed `5.0 mm` fallback, the center-bounded rule
is deliberately not a superset in every geometry: a pose that is within
`5.0 mm` of its target but already above the live object center becomes a
rejection. No successful object trial has been reported as depending on that
condition. The pre-existing stable trials occurred before this experimental
fallback and remain protected by the unchanged ordinary path.

## Diagnostics

The shared Z decision reports:

- signed and absolute Z error;
- ordinary negative and positive tolerance (`3.0 mm`);
- bottom, center, and top when available;
- effective positive limit (`center_z - target_z`) when valid;
- decision mode: `ordinary`, `controlled_center`, or `rejected`;
- a precise reason for missing/invalid bounds, target outside bounds, actual
  below bottom, actual above center, or a negative residual beyond tolerance.

Controlled success logs explicitly state that no additional lower command was
sent.

## Verification

Add deterministic regression coverage proving:

- ordinary final success at positive and negative residuals within `3.0 mm`
  remains independent of bounds;
- the ordinary path does not change the command or close sequence;
- a final positive residual above `3.0 mm` passes after settling when actual Z
  is between bottom and center and XY is within `2.0 mm`;
- the current `gelatin_box` values (`+5.2 mm`, actual below center) pass;
- actual Z exactly at center passes;
- actual Z above center fails even when still below top;
- actual Z below bottom fails;
- target Z outside the live bounds fails;
- missing, non-finite, reversed, or zero-height bounds disable only the
  fallback;
- a final negative residual beyond `3.0 mm` fails;
- non-final waypoints remain symmetric at `3.0 mm`;
- non-vertical profiles never use the fallback;
- the final waypoint and close-readiness checks use the same predicate;
- close is sent only after both final gates pass;
- the fixed `5.0 mm` configuration is absent from runtime configuration and
  current operator documentation; the superseded design remains as historical
  rationale.

Run focused configuration, executor, and trajectory-planner tests; the related
grasp regression suite; Python compilation; fatal flake8 on changed Python
files; and a Docker `colcon build --symlink-install` for `my_course_pkg`.
Confirm the installed runtime retains `1.5 mm` pregrasp XY, `2.0 mm` descent XY,
and `3.0 mm` ordinary Z.

Live validation must not be launched automatically. After restarting the
experiment launch, run a fresh close-only `gelatin_box` trial with
`GRASP_DEBUG_STOP_AFTER_CLOSE=1`. Require a `controlled_center` success, the
close result, and the immediate debug stop. Because `sponge`, `foam_brick`, and
`rubiks_cube` are already reported stable, automated non-regression coverage is
required before any optional repeat motion test.

## Non-Goals

- Changing grasp geometry or making the final target higher.
- Relaxing downward/table-side motion.
- Fixing `pudding_box` planning or IK failures.
- Changing `side`, `round_top`, `centered`, or `top_down` behavior.
- Tuning the gripper or perception pipeline.
- Automatically starting simulator or robot motion.

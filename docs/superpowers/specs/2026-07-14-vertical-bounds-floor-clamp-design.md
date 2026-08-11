# Vertical Bounds Floor Clamp Design

## Goal

Prevent any tabletop `vertical` grasp from commanding its TCP below a generic
height derived from the selected object's live world-axis bounds. Preserve a
valid higher grasp-library target, avoid object-name special cases, and leave
all other grasp profiles unchanged.

This is a planner correction, not an execution-gate relaxation. The existing
1.5 mm XY gate, 3 mm Z gate, 5 mm descent spacing, 1.5-second waypoint timeout,
pregrasp calibration, frozen command offset, and final verification remain
unchanged.

## Live Evidence

The latest foam-brick run passed pregrasp calibration and descent waypoints
1-15. Waypoint 16 targeted nominal Z=0.8913 m but timed out at actual Z=0.8987
m with only 0.3 mm XY error and 7.4 mm Z error. The screenshot showed the brick
centered between the open fingers with both fingertips at table height.

The live target bounds were:

```text
center_z = 0.9160 m
half_height = 0.0260 m
bounds_bottom_z = 0.8900 m
```

The grasp library requested final TCP Z=0.8713 m, 18.7 mm below the live object
bottom, and would have required another 27.4 mm descent from the physically
reached pose. The failure is an invalidly low nominal target, not lateral
misalignment or insufficient settling time.

## Considered Approaches

### Generic live-bounds floor clamp

Keep the grasp-library Z whenever it is already high enough. Otherwise raise it
to a minimum defined from the live target-bounds bottom plus a gripper/TCP
clearance. This is selected because it is generic across `vertical` tabletop
objects, preserves valid library geometry, and prevents below-surface targets.

### Object-specific foam-brick offset

Add approximately 25 mm only for `foam_brick`. This directly fixes the recorded
trial but encodes the object name and does not protect other vertical objects
from the same library/bounds disagreement. It is not selected.

### Close when descent stalls

Treat a Z timeout with acceptable XY as contact and close at the observed pose.
This is runtime-generic but cannot distinguish harmless table contact from an
object-top collision, another obstacle, or a controller fault. It is not
selected.

## Height Rule

Add a validated configuration value:

```text
GRASP_VERTICAL_MIN_TCP_ABOVE_TARGET_BOTTOM_M = 0.008
```

It must be finite and non-negative. An environment override with the same name
is allowed for future tool calibration, but the default is the validated
behavior.

For every selected `vertical` grasp candidate after loading the required live
target bounds, calculate in world coordinates:

```text
bounds_bottom_z = target_bounds.center[2] - target_bounds.half_height
minimum_tcp_z = bounds_bottom_z
                + VERTICAL_MIN_TCP_ABOVE_TARGET_BOTTOM_M
adjusted_grasp_z = max(library_adjusted_grasp_z, minimum_tcp_z)
z_correction = adjusted_grasp_z - library_adjusted_grasp_z
```

The `/scene_clearance_bounds` publisher supplies complete world-axis AABBs with
identity marker orientation, so `center_z - half_height` is the exact live
world-Z minimum even when the simulated object body is rotated.

Equality passes without correction. The clamp only raises a target; it never
lowers one. With the recorded foam-brick values:

```text
minimum_tcp_z = 0.9160 - 0.0260 + 0.0080 = 0.8980 m
z_correction = 0.8980 - 0.8713 = +0.0267 m
```

This is approximately the user-approved 25 mm reduction in descent and matches
the observed reachable Z=0.8987 m within the existing 3 mm gate.

## Planner Data Flow

The planner already requires one fresh target bounds marker for every
`vertical` grasp and already centers candidate XY on `target_bounds.center`.
Extend that same bounds-alignment helper as follows:

1. Validate the candidate 6D pose, transform, target center, positive finite
   `half_height`, and configured clearance.
2. Preserve the current exact XY-center correction.
3. Apply the one-sided Z floor clamp to both the copied 6D pose and copied
   transform translation.
4. Preserve orientation and input immutability.
5. Return the full XYZ correction for diagnostics.
6. Build pregrasp, lift, return, release-clearance, and retreat poses from the
   adjusted final grasp through the existing trajectory planner.

No additional perception topic or table-plane estimator is introduced. If the
required target bounds are missing or invalid, existing fail-closed behavior
remains in force.

## Component Changes

### `config.py`

- Add `read_vertical_min_tcp_above_target_bottom_m()`.
- Default the matching environment setting to 0.008 m.
- Reject NaN, infinity, and negative values.

### `pick_place_planner.py`

- Import the validated vertical floor clearance.
- Extend `_center_vertical_grasp_candidate()` to apply the generic one-sided Z
  clamp and return an XYZ correction.
- Expand the `Vertical grasp centering` diagnostic with original Z,
  bounds-bottom Z, minimum TCP Z, Z correction, and adjusted Z.
- Do not inspect or branch on `selected_object_name` when applying the rule.

### Selector, executor, simulator, and other profiles

No change. Grasp-library loading, object classification, bounds publication,
feedback calibration, descent gates, gripper behavior, and `side`, `top_down`,
and default profiles remain unchanged.

## Error Handling

The planner fails before motion if the target center, half-height, configured
clearance, derived bottom, minimum Z, adjusted pose, or transform is non-finite,
or if the half-height is not positive. It must not silently fall back to the
old below-floor target.

The executor continues to own runtime convergence failures. A corrected target
that still cannot meet the existing physical gates times out and holds exactly
as it does now.

## Tests

Configuration and planner tests will prove:

- the clearance defaults to exactly 0.008 m;
- a finite non-negative environment override is honored;
- NaN, infinity, and negative values are rejected;
- the recorded foam-brick values clamp 0.8713 m to 0.8980 m;
- a library target already above the minimum is preserved exactly;
- equality at the minimum is preserved;
- the same bounds rule applies without checking the object name;
- XY is still centered exactly while orientation and inputs are unchanged;
- the copied transform translation matches the adjusted 6D pose;
- invalid or non-positive target-bounds height fails closed;
- an integration-level vertical plan derives its pregrasp and downstream poses
  from the adjusted final Z;
- non-vertical plan behavior is unchanged.

Run focused configuration/planner tests, the related grasp regression suite,
Python compilation, and `colcon build --packages-select my_course_pkg
--symlink-install` in the Docker-mounted workspace. Confirm modified runtime and
test files match between both host workspaces apart from line endings.

## Live Validation

Do not launch robot motion automatically. After rebuild and simulator restart,
the first user-run foam-brick trial must use:

```bash
GRASP_DEBUG_STOP_AFTER_CLOSE=1 PYTHONUNBUFFERED=1 \
ros2 run my_course_pkg grasp_demo
```

The qualifying log must show a bounds bottom near 0.890 m, a minimum/final TCP
near 0.898 m, no commands for the former lower waypoints, all remaining
waypoints passing the unchanged gates, and a close with the brick visibly
between both fingers. Lift and return validation resume only after that bounded
close succeeds.

## Acceptance Criteria

- No object-name-specific Z correction is introduced.
- Every `vertical` target is clamped to at least live bounds bottom plus 8 mm.
- Higher valid library targets remain unchanged.
- XY centering, orientation, and all execution safety gates remain unchanged.
- Missing or invalid required geometry fails before motion.
- Focused tests, related regression, compilation, and package build pass.

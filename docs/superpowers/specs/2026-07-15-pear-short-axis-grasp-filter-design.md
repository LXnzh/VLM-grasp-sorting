# Pear Short-Axis Grasp Filter Design

## Goal

Make `pear` use a top-down grasp whose Robotiq closing axis follows the pear's
short horizontal axis, so the fingers descend along the long sides and the
object fits inside the verified effective opening.

The change is deliberately object-specific. All non-pear objects must retain
their current candidate generation, filtering, ranking, planning, trajectory,
and execution behavior.

## Safety And Isolation Principle

The existing behavior is already stable for the other objects. Therefore this
work uses an exact-name opt-in policy:

```text
normalized object name == "pear" -> pear-only direction/width filter
all other object names           -> existing code path unchanged
```

This work must not change shared `ROUND_TOP_*`, `SIDE_GRASP_*`, or
`VERTICAL_*` defaults, category/profile mappings, planner tolerances, approach
clearance, executor convergence, gripper commands, or gripper effort.

Pure geometry helpers may be reusable, but only the exact `pear` branch calls
the new policy in this change. `tuna_fish_can` and `pudding_box` will be handled
by separate designs and changes.

## Observed Failure

The latest pear trial selected a nearly vertical top-down grasp and reached the
object, but a finger contacted the pear before the TCP reached the commanded
final pose. The close command was never reached.

The failure is directional, not a lack of top-down candidates:

- The Robotiq finger closing axis is TCP `X`.
- Pear's stored horizontal bounding-box dimensions are `66.546 x 100.455 mm`.
- The selected candidate's closing axis was about `69.3 degrees` away from the
  pear short axis.
- Its mesh-projected required width was about `97.4 mm`, exceeding the verified
  effective opening of `85.16 mm` by about `12.2 mm`.
- The shared round-top selector instead reported `66.546 mm` because it always
  uses `min(bbox_x, bbox_y)`, independent of the candidate's closing direction.

That scalar minimum is valid only when the fingers are already aligned with the
short axis. It allowed a long-axis-spanning grasp to rank first.

## Offline Candidate Evidence

The existing pear library contains `7,466` raw poses. The current 45-degree yaw
expansion produces `59,728` poses. Replaying the latest scene pose through the
current orientation, center, and normalized-height gates leaves about `325`
candidates.

Adding the proposed pear-only conditions leaves two candidates:

| Property | Candidate 1 | Candidate 2 |
|---|---:|---:|
| Raw library index | 5675 | 5675 |
| Object-center yaw expansion | 180 deg | 0 deg |
| Short-axis alignment error | 4.858 deg | 4.858 deg |
| Conservative projected AABB width | 74.81 mm | 74.81 mm |
| Effective-opening margin | 10.35 mm | 10.35 mm |
| World-down approach error | about 16 deg | about 19 deg |
| Geometry-center offset | 9.91 mm | 9.91 mm |
| Normalized height | 0.0032 | 0.0032 |

The candidates differ by a 180-degree wrist orientation and lie on opposite
sides of the pear's long-axis center. This provides two planner attempts without
inventing a new grasp or loading the mesh at runtime.

## Candidate Geometry Contract

For a candidate transform `T_object_grasp`, the TCP `X` axis expressed in the
object frame is the first rotation column:

```text
closing_axis_object = T_object_grasp.rotation[:, 0]
```

Only its horizontal projection is used for pear footprint alignment:

```text
closing_xy = normalize(closing_axis_object[:2])
```

The pear short horizontal axis is derived from its stored bounding-box size,
not from a scene coordinate or a hard-coded world yaw:

```text
short_axis_index = argmin(bbox_size[:2])
```

Closing direction is sign-symmetric, so `+short_axis` and `-short_axis` are
equivalent. The alignment error is:

```text
error = acos(abs(dot(closing_xy, short_axis)))
```

The conservative required opening for the axis-aligned horizontal bounding box
is the support width along the closing direction:

```text
required_width =
    abs(closing_xy.x) * bbox_size.x
  + abs(closing_xy.y) * bbox_size.y
```

The candidate passes only when both conditions hold:

```text
alignment_error <= 5 degrees
effective_opening - required_width >= 5 mm
```

A near-zero horizontal projection is invalid and fails closed, even though it
is not expected for a valid top-down candidate.

## Selector Data Flow

The existing round-top pipeline remains responsible for loading the library,
yaw expansion, world-down approach filtering, geometry-center filtering,
normalized-height filtering, table clearance, scoring, and candidate count.

Immediately after the existing normalized-height gate:

1. If the normalized object name is not exactly `pear`, run the current scalar
   width gate with no behavior change.
2. If it is `pear`, compute alignment error and projected AABB width for every
   remaining candidate.
3. Reject candidates above the pear-only 5-degree alignment limit.
4. Reject candidates with less than the pear-only 5 mm opening margin.
5. Run the existing table-clearance gate.
6. Rank survivors with the existing round-top score and return up to the
   existing round-top candidate limit.

The score is not changed. Direction and physical fit are hard validity gates,
not soft preferences that an unsafe candidate could overcome with a better
approach score.

## Configuration

Use pear-namespaced constants only:

```text
PEAR_MAX_CLOSING_AXIS_ERROR_DEG = 5.0
PEAR_MIN_OPENING_MARGIN_M = 0.005
```

The existing `ROUND_TOP_MAX_GRIPPER_OPENING_M = 0.08516` remains the source of
the verified effective opening. No shared default is changed.

The stored pear geometry also remains unchanged:

```text
center    = [-0.033320, 0.017995, 0.032652]
bbox_size = [ 0.066546, 0.100455, 0.065663]
```

## Diagnostics

Pear selection logs report:

- candidates entering the pear-only gate;
- candidates remaining after short-axis alignment;
- candidates remaining after opening-margin filtering;
- each selected candidate's alignment error, projected required width, and
  opening margin;
- the existing approach, center, height, table-clearance, score, and planner
  diagnostics.

The existing non-pear log format and values remain unchanged.

## Failure Behavior

If no pear candidate satisfies direction, width margin, and the existing safety
filters, selection fails before motion with a pear-specific reason. It must not
fall back to the old generic round-top ranking, relax a shared threshold,
synthesize an unvalidated yaw, approach blindly, or continue to gripper close.

Planner rejection remains unchanged: the existing planner tries the two
selected candidates in order. If neither is reachable or scene-clear, the task
fails through the current no-candidate path.

## Code Boundaries

- `grasp/config.py`: add only the two pear-namespaced constants.
- `grasp/grasp_selector.py`: add pure closing-axis/alignment/projected-width
  helpers and the exact-name pear gate inside the round-top branch.
- `test/test_grasp_selector.py`: add pear geometry, routing, filtering, and
  non-pear regression tests.
- Planner, trajectory, executor, perception, profile mapping, and gripper code
  are out of scope.

## Verification

### Automated

1. Verify TCP `X` aligned with pear `+X` or `-X` has zero sign-symmetric error.
2. Verify TCP `X` aligned with pear long axis is rejected.
3. Verify the projected AABB formula returns `66.546 mm` on the short axis and
   `100.455 mm` on the long axis.
4. Verify the pear filter rejects a current-failure-like long-axis candidate.
5. Verify the pear filter retains both safe 180-degree alternatives when they
   pass the existing round-top gates.
6. Verify pear candidate exhaustion fails closed before planning or execution.
7. Verify non-pear round-top objects do not call the pear filter and preserve
   their pre-change selected matrices and ordering for fixed inputs.
8. Run the complete grasp-selector and pick-place-planner focused test suites.

### Offline Real-Library Check

Replay the current `grasps/016_pear` library through the implemented selector
using the recorded pear scene pose. Confirm:

- the bad approximately 69-degree candidate is absent;
- every selected candidate is within 5 degrees of the short axis;
- every selected candidate has at least 5 mm conservative opening margin;
- the two known index-5675 / 0-and-180-degree candidates remain available.

### Staged Live Check

1. Run perception and selection only; inspect the candidate metrics.
2. Move to pregrasp and stop.
3. Descend to final grasp without closing; visually confirm the two fingers lie
   along the pear's long sides and neither finger is stopped on top of it.
4. Close without lifting and verify bilateral contact.
5. Lift and hold.
6. Complete three fresh-scene grasp-and-lift trials before declaring pear
   stable.

No live stage may be used to justify changing shared behavior for another
object.

## Acceptance

- `pear` no longer selects a grasp that spans its long horizontal dimension.
- Every selected pear candidate has at most 5 degrees short-axis alignment
  error and at least 5 mm conservative opening margin.
- The existing library supplies two 180-degree alternatives to the planner.
- Pear completes three consecutive fresh-scene grasp-and-lift trials.
- All non-pear candidate results and focused regression tests remain unchanged.
- No planner, executor, perception, profile mapping, or gripper behavior changes
  are included in the pear patch.

## Alternatives Not Chosen

- **Change round-top width handling globally:** geometrically desirable, but it
  could reorder or reject already-stable fruit and ball grasps. It violates the
  isolation principle for this repair.
- **Add a global yaw preference:** the other round objects are approximately
  symmetric and already stable; a shared preference adds unnecessary risk.
- **Use only a score penalty:** an invalid wide grasp could still win, so fit and
  direction remain hard gates.
- **Synthesize a new arbitrary pear yaw:** unnecessary because the library plus
  existing symmetry expansion already contains two safe poses.
- **Load the pear mesh at runtime:** exact mesh support width would be less
  conservative but adds dependencies and runtime failure modes. The static AABB
  leaves more than 10 mm margin for the known safe candidates.

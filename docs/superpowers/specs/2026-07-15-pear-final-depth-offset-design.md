# Pear Final-Depth Offset Design

## Goal

Raise only `pear` final and pregrasp targets by `30 mm` relative to the current
shared round-top behavior, so the robot closes from the visually validated
straddling pose instead of repeatedly commanding an unreachable deeper target.

All non-pear object offsets, candidate selection, planner tolerances, executor
convergence, gripper commands, and scene-clearance behavior remain unchanged.

## Live Evidence

Trial `/tmp/my_course_experiment_sessions/20260715_181545_780230/trial_001`
validated the pear short-axis direction:

- exactly two candidates survived the pear-only direction and opening gates;
- both had `4.858 deg` short-axis error and `10.35 mm` opening margin;
- the screenshot showed the open fingers straddling the pear's long sides;
- scene approach clearance kept both candidates.

Candidate 1 then plateaued before close:

```text
target xyz = [-0.2131, -0.9427, 0.9028] m
actual xyz = [-0.2047, -0.9490, 0.9326] m
actual-target = [+8.4, -6.3, +29.8] mm
XY error = about 10.5 mm
total error = 31.6 mm
```

The dominant residual was upward Z, it remained stable through repeated final
commands, and no close command ran. The existing shared `GRASP_Z_OFFSET=-0.020`
had lowered the raw library target from about `0.9228 m` to `0.9028 m`.

## Approved Strategy

Add one exact-name pear offset:

```text
PEAR_GRASP_Z_OFFSET = +0.010 m
```

Compared with the current shared `-0.020 m`, this raises pear targets by
`30 mm`. For the recorded trial, candidate 1 would move from `0.9028 m` to
`0.9328 m`, within `0.2 mm` of the visually validated held height.

The offset is applied in world Z through the existing
`select_grasp_pose_candidates_6d` path. Candidate object-frame transforms,
closing direction, score, ordering, and orientation do not change.

## Isolation Boundary

`get_grasp_z_offset` normalizes the object name and applies this order:

```text
pear             -> PEAR_GRASP_Z_OFFSET
side profile     -> existing SIDE_GRASP_Z_OFFSET
vertical profile -> existing VERTICAL_GRASP_Z_OFFSET
top_down profile -> existing TOP_DOWN_GRASP_Z_OFFSET
all remaining    -> existing GRASP_Z_OFFSET
```

Only exact normalized `pear` takes the new first branch. Apple, lemon, peach,
orange, plum, baseball, tennis ball, and racquetball remain on the shared
round-top `GRASP_Z_OFFSET=-0.020 m` path.

This change must not modify:

- `GRASP_Z_OFFSET` or any shared profile default;
- `FINAL_APPROACH_POSITION_TOLERANCE_M=0.018`;
- executor retry, convergence, or close-readiness logic;
- pear direction/width limits and candidate ordering;
- planner approach distances or obstacle clearance;
- any tuna, pudding, vertical, side, centered, or top-down path.

## Data Flow

1. The existing pear-only selector returns the same two safe object-frame
   candidates.
2. `get_grasp_z_offset("pear")` returns `+0.010 m`.
3. `select_grasp_pose_candidates_6d` adds this value only to world Z.
4. The planner derives the pregrasp from the raised final pose, so the complete
   approach segment shifts upward consistently.
5. Scene-clearance, MoveIt planning, interpolation, and executor convergence
   run unchanged.
6. Close is sent only if the existing `18 mm` three-dimensional final-pose gate
   succeeds.

The selector's table-clearance calculation already calls
`get_grasp_z_offset`, so it evaluates the same raised pear endpoint and becomes
more conservative against downward table contact.

## Configuration

Add a pear-namespaced environment setting with a reviewed default:

```text
GRASP_PEAR_Z_OFFSET=0.010
```

It maps to `PEAR_GRASP_Z_OFFSET`. The default is part of the pear policy, not a
shared tuning knob. Values are logged through the existing prepared-candidate
`z_offset` diagnostic.

## Failure Behavior

The change does not introduce a fallback acceptance path. If the raised target
still fails the existing final-pose convergence gate, execution stops before
close exactly as it does today.

If staged validation shows the raised pose is too high for bilateral contact,
adjust only `GRASP_PEAR_Z_OFFSET` using measured evidence. Do not compensate by
relaxing the global final tolerance or changing another object's offset.

## Code Boundaries

- `grasp/config.py`: add `PEAR_GRASP_Z_OFFSET` only.
- `grasp/grasp_selector.py`: add the exact normalized pear branch in
  `get_grasp_z_offset`.
- `test/test_grasp_selector.py`: add pear offset, non-pear offset, final-pose,
  and real-library invariance coverage.
- Planner, executor, trajectory, perception, gripper, and profile mappings are
  out of scope.

## Automated Verification

1. `get_grasp_z_offset("pear")` and normalized spelling variants return
   `+0.010 m`.
2. Every configured non-pear object returns its exact pre-change offset.
3. Pear's prepared final world Z is `30 mm` above the pre-change target while
   XY and orientation are unchanged.
4. The real pear library still returns only raw index 5675 at yaw 0/180 with
   unchanged `4.858 deg` alignment and `10.35 mm` opening margin.
5. Representative apple candidate matrices, ordering, and `-0.020 m` offset
   remain unchanged.
6. Existing side, vertical, centered, top-down, selector, planner, trajectory,
   and executor regressions pass.
7. Host and Docker runtime/test file hashes match; the symlink package build and
   installed constants pass.

## Staged Live Validation

1. Restart the experiment launch so the new default is imported.
2. Run pear with `GRASP_DEBUG_STOP_AT_PREGRASP=1`; require the logged candidate
   `z_offset=0.0100 m` and collision-free pregrasp.
3. Run with `GRASP_DEBUG_STOP_AT_GRASP=1`; require final Z near `0.9328 m`, both
   fingers along the long sides, and no table/object penetration.
4. Run with `GRASP_DEBUG_STOP_AFTER_CLOSE=1`; require bilateral contact and no
   object ejection.
5. Run lift-and-hold, then three fresh-scene full grasp-and-lift trials.

## Acceptance

- Runtime logs `z_offset=0.0100 m` for pear and retains the same two safe
  candidates.
- The final approach reaches the raised target within the existing `18 mm`
  three-dimensional tolerance without repeated downward plateauing.
- Pear closes with both fingers on its long sides and completes three
  consecutive fresh-scene grasp-and-lift trials.
- Every non-pear offset and selector result remains unchanged.
- No executor, planner, perception, trajectory, gripper, or other-object change
  is present in the implementation patch.

## Alternatives Not Chosen

- **Accept the old stalled pose with a pear-only convergence exception:** the
  controller would still press toward a known over-deep target before the
  exception; this hides the target error and adds executor complexity.
- **Relax the global final tolerance to about 32 mm:** this could accept the
  recorded `10.5 mm` lateral error for every object and violates the isolation
  principle.
- **Edit or regenerate the pear candidate library:** the selected directions
  are already correct; changing object-frame poses is unnecessary and harder to
  regress.

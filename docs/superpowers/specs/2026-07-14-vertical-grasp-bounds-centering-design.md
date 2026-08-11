# Vertical Grasp Bounds-Centering Design

## Goal

Correct the physical top-down grasp position for box-like objects that use the
`vertical` grasp profile. The final tool-center-point (TCP) X/Y position must be
centered on the selected object's live `/scene_clearance_bounds` marker before
any arm motion is planned.

This is a profile-level correction, not a `foam_brick` special case. It applies
to every object currently mapped to `vertical`, including `pudding_box`,
`gelatin_box`, `sponge`, `foam_brick`, and `rubiks_cube`.

## Evidence and Problem Statement

The failed foam-brick run selected the correct mask and produced a plausible
FoundationPose orientation, but its only library grasp targeted world X/Y
`[-0.4684, -0.9396]`. The pre-run live brick-bounds center was approximately
`[-0.4996, -0.9391]`, leaving the requested TCP about 31 mm toward one X edge.
The gripper contacted or nudged the edge and the brick did not lift.

Tightening the existing close-result threshold would only report this miss more
accurately. It would not correct the down-grasp motion. The requested change
therefore modifies the planned execution target itself.

## Selected Approach

Keep the grasp library's selected vertical orientation and Z height, then apply
one world-frame X/Y translation to the selected candidate so its final TCP X/Y
equals the target bounds center.

For a selected vertical candidate:

```text
correction_xy = target_bounds_center_xy - original_final_tcp_xy
centered_final_tcp_xy = original_final_tcp_xy + correction_xy
```

The planner receives the centered final pose. Its existing pregrasp generation
therefore shifts by the same world X/Y correction relative to the original
trajectory. Any small X/Y component caused by the retained approach angle is
preserved; the change does not force pregrasp X/Y to equal final-pose X/Y. The
implementation must keep the two representations of the candidate, the 6D
grasp pose and `T_world_grasp`, consistent.

The latest foam-brick data would produce a correction of approximately
`[-0.0312, +0.0005]` m. No object name, slot coordinate, or measured correction
value is hard-coded.

## Data Flow and Component Boundaries

1. Perception and the current grasp selector run as they do today to determine
   the selected object, grasp profile, vertical candidate orientation, and Z.
2. When and only when the resolved profile is `vertical`, the planner waits for
   the next `/scene_clearance_bounds` `MarkerArray` using the existing scene
   timeout.
3. A target-bounds helper normalizes marker and selected-object names using the
   existing name rules, finds exactly one target marker, transforms its pose
   into the `world` planning frame, and validates finite positive geometry.
4. A separate candidate-centering helper copies the selected candidate and
   replaces only its world X/Y translation with the target marker's world X/Y
   center. It preserves Z and the full rotation.
5. Existing pregrasp, lift, hold, return, release, and retreat construction then
   operates on the corrected candidate without a new executor branch.

Target loading is separate from non-target obstacle filtering. Existing side
and round-top clearance behavior must continue to exclude the target object
from the obstacle list.

## Failure Handling

Vertical planning fails closed before sending any MoveIt or gripper command if:

- no marker array arrives before the existing timeout;
- no marker matches the selected object after name normalization;
- more than one marker matches the selected object;
- the marker scale, pose, or transformed world center is invalid or non-finite;
- the marker frame cannot be transformed into `world`.

There is no fallback to the original off-center perception/library X/Y target.
The error message must identify the selected object and the bounds failure.

The planner logs the object name, original candidate X/Y, target-bounds X/Y,
and applied correction X/Y so a live run can prove which motion target was used.

## Scope Boundaries

This change deliberately does not:

- add a `foam_brick`-specific branch or fixed offset;
- change side, round-top, centered, or hammer/top-down grasp selection;
- replace FoundationPose orientation or Z with simulator geometry;
- change final-approach convergence tolerances;
- change the generic gripper close threshold;
- add object-lift verification or alter success reporting;
- alter lift, hold, return, release, or retreat behavior.

Close-result and lift verification remain useful follow-up safety work, but they
are independent of the execution-position correction requested here.

The live bounds are simulator geometry already published for planning safety.
This design is consequently simulation-specific and does not claim that the
same source will be available on a physical robot.

## Tests

Focused unit and planner tests must cover:

- target-name normalization and selection of exactly one target marker;
- transformation of a target marker from its header frame into `world`;
- rejection of missing, duplicate, invalid, and untransformable target bounds;
- exact replacement of a vertical candidate's final TCP X/Y by the bounds
  center;
- preservation of candidate Z and rotation;
- consistency between the corrected 6D pose and `T_world_grasp`;
- translation of the generated vertical pregrasp by the same correction delta,
  while retaining its existing approach-vector offset from the final pose;
- no bounds-centering change and no new bounds requirement for non-vertical
  profiles;
- preservation of existing non-target obstacle parsing and clearance tests;
- the recorded foam-brick regression, where the planned final X shift is about
  -31 mm and the resulting final TCP is centered on the marker.

The related selector, planner, trajectory, executor, and evaluation regression
suite must remain green, followed by a `my_course_pkg` rebuild in the active
Docker-mounted workspace.

## Live Validation

After rebuilding, restart the full simulator launch, verify unique arm and
gripper servers, reset the scene, and run a fresh foam-brick perception pipeline.
Real-motion validation uses `GRASP_DEBUG_STOP_AFTER_LIFT=1` so the arm completes
the lift and hold but does not execute the return/release path.

A qualifying run must show all of the following:

- the verifier still selects the real foam brick;
- the centering log names `foam_brick` and shows the final TCP X/Y equal to the
  fresh target-bounds X/Y;
- the arm descends through the brick center rather than an edge;
- the brick is visibly captured between the fingers and rises with the arm;
- no neighboring object is contacted before the bounded after-lift stop.

The project-wide experiment policy requires two consecutive independent
qualifying runs. Each run starts from a restarted full launch, reset scene, and
fresh perception result. Only after both bounded validations should a normal
lift-return trial be run.

## Acceptance Criteria

The change is complete when the profile-generic tests pass, the package builds,
and two consecutive independent bounded foam-brick trials visibly lift the
brick with the logged TCP centered on its live bounds. A stricter success
criterion is not part of this acceptance gate because this design corrects
execution rather than reporting.

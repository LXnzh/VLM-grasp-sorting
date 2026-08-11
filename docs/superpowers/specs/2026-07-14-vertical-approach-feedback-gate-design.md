# Vertical Approach Feedback-Gate Design

## Goal

Prevent an open gripper finger from striking the top of a box-like target while
executing a `vertical` grasp. Preserve the selected grasp orientation and the
existing live-bounds X/Y centering, but require the arm to prove that it is
laterally aligned before descending and while progressing through the descent.

This is a `vertical` profile execution policy, not a `foam_brick` object-name
special case. Other grasp profiles keep their current execution path.

## Evidence and Problem Statement

The latest foam-brick run proved that live-bounds centering was active:

- target bounds center X/Y was approximately `[-0.4996, -0.9392]` m;
- the selected library pose was corrected by approximately
  `[-0.0312, +0.0004]` m;
- the planned final TCP X/Y matched the target-bounds center.

The remaining failure happened during execution. After the current Cartesian
approach had already descended, feedback reported an initial 3D position error
of about 27.1 mm. One correction command reduced it to about 12.6 mm, which the
global 18 mm final-approach tolerance accepted. By then one open finger had
already reached the brick's top surface instead of passing beside it.

This brick presents approximately 80.4 mm across the current gripper closing
direction, while the gripper opens to approximately 85 mm. The nominal total
clearance is therefore only about 4.6 mm, or 2.3 mm per finger. A 12.6 mm final
position error is physically incompatible with this grasp even though the old
software gate accepts it.

The current `move_linear()` implementation also sends every interpolated pose
without waiting for feedback at each pose. With a 100 mm approach and the
global 20 mm waypoint limit, several descent targets can be queued before the
arm has settled. The only feedback convergence check occurs after the entire
descent, which is too late to prevent contact.

## Considered Approaches

### Pregrasp-only reanchor

Reissue the pregrasp target at the safe hover height and start the existing
descent after it converges. This is the smallest useful change, but it cannot
detect tracking drift or queued-target lag during the following 100 mm descent.

### Fixed X/Y compensation

Add another constant offset to the centered target. This can hide one observed
miss but cannot correct run-to-run controller error. It would also undo the
meaning of the live-bounds center and is rejected.

### Pregrasp reanchor plus feedback-gated descent

Actively converge at the safe pregrasp, then descend through short waypoints
one at a time. Each waypoint must be confirmed from live pose feedback before
the next lower target is issued. This directly addresses both observed failure
modes and is the selected approach.

Changing the selected grasp to a synthesized 90-degree short-side orientation
is intentionally not part of this design. It may be a later robustness option,
but the requested fix keeps the currently accepted grasp orientation.

## Selected Execution Sequence

The special path applies only when both conditions are true:

- the plan's resolved `grasp_profile` is `vertical`;
- the executing step is the initial `approach_grasp` step.

The sequence is:

1. Execute the existing open-gripper and collision-aware MoveIt transfer to
   `move_to_pre_grasp`.
2. At the safe pregrasp height, enter Cartesian control and actively reissue
   the exact planned pregrasp pose until live feedback is within the vertical
   X/Y and Z tolerances. No downward waypoint may be issued before this passes.
3. Interpolate from the planned pregrasp to the planned final grasp using a
   vertical-specific maximum Cartesian spacing of 5 mm. Do not execute the
   duplicated start point because the pregrasp has already been verified.
4. Before each next lower waypoint, confirm that the current TCP remains within
   the X/Y tolerance of the approach line. If it is already outside tolerance,
   stop without issuing that lower waypoint.
5. Issue exactly one lower waypoint, then poll live TCP feedback. The waypoint
   passes only when both its X/Y error and absolute Z error meet their limits.
   The next lower waypoint is not issued until this happens.
6. If X/Y leaves tolerance after a lower command, replace the outstanding
   descent target with a hold command at the observed current pose and fail the
   approach. Do not attempt a lateral correction near the target.
7. If Z does not converge before the timeout, hold the observed current pose
   and fail the approach.
8. Recheck the strict vertical X/Y and Z limits at the final grasp pose
   immediately before the gripper-close step. Any failure blocks gripper close,
   lift, return, and release.

The planned orientation is included unchanged in every commanded pose. This
design does not add a new orientation-selection rule or rotate the grasp.

## Tolerances and Configuration

Add validated configuration values:

```text
GRASP_VERTICAL_APPROACH_XY_TOLERANCE_M=0.0015
GRASP_VERTICAL_APPROACH_Z_TOLERANCE_M=0.003
GRASP_VERTICAL_APPROACH_WAYPOINT_MAX_DIST_M=0.005
```

The X/Y gate uses the Euclidean world-frame lateral error. A 1.5 mm 2D bound
also bounds the error projected onto any horizontal gripper closing axis to at
most 1.5 mm. For the observed 2.3 mm nominal per-finger clearance, this leaves
some positive geometric margin instead of accepting a centimeter-scale miss.

The 3 mm Z limit prevents the executor from advancing through multiple lower
targets while the arm is vertically behind. The 5 mm waypoint spacing bounds
how far one accepted command can lower the arm before another feedback gate.

All three values must be finite and strictly positive. The new path reuses the
existing final-approach settle timeout and command period so the change does not
introduce redundant timing controls. Environment variables remain available
for measured tuning without code changes.

## Component Boundaries

### `config.py`

Owns parsing and validation of the three vertical-approach values. Invalid zero,
negative, NaN, or infinite values fail at startup.

### `executor.py`

Owns all feedback-sensitive behavior:

- profile and initial-approach detection;
- safe-height pregrasp convergence;
- vertical-specific waypoint generation;
- one-waypoint-at-a-time command/feedback sequencing;
- strict final verification;
- hold-and-fail behavior and diagnostic logging.

The trajectory planner continues to describe the same pregrasp and final grasp
poses. It does not need to encode controller feedback or add synthetic plan
steps.

### Existing generic path

The generic `move_linear()` method and the 18 mm global final-approach tolerance
remain unchanged for non-vertical profiles. The vertical initial approach must
not fall through to that permissive final gate after completing its strict
endpoint verification.

## Failure Handling and Diagnostics

Introduce a vertical-approach execution error that records:

- stage (`pregrasp`, waypoint index, or `final_verify`);
- target and actual TCP poses;
- X/Y error and absolute Z error;
- configured X/Y and Z tolerances;
- whether the failure was lateral deviation or timeout.

Every successful gate logs the profile, stage, waypoint index/count, target and
actual XYZ, X/Y error, Z error, and command count. On failure, the executor logs
the same values, commands the observed pose as a hold target when Cartesian
control is active, and raises the error. It must never continue to gripper close
or retry another grasp candidate after a descent-tracking failure.

The pregrasp convergence may actively correct X/Y because it occurs at the
configured pregrasp clearance (100 mm with the current default). Once descent
starts, the policy is fail-closed rather than laterally correcting beside or
against the object.

## Debug-Stop Behavior

`GRASP_DEBUG_STOP_AT_PREGRASP=1` stops only after the new safe-height pregrasp
convergence has passed, so the debug pose represents the verified starting
condition for descent.

`GRASP_DEBUG_STOP_AT_GRASP=1` and `GRASP_DEBUG_STOP_AFTER_CLOSE=1` retain their
current meanings, but the strict vertical endpoint verification runs before
either stop can report a valid final grasp state or allow gripper close.

## Scope Boundaries

This change deliberately does not:

- alter the live-bounds centering calculation;
- rotate, synthesize, or re-rank grasp candidates;
- add a `foam_brick`-specific offset or branch;
- change grasp Z selection or gripper opening/closing targets;
- change side, centered, round-top, hammer, or legacy motion behavior;
- add lift-success perception or change final success reporting;
- modify transfer, lift, return, release, or retreat trajectories.

## Tests

Configuration tests must cover defaults and rejection of zero, negative,
non-finite, and malformed values.

Executor tests must prove:

- a vertical plan actively converges at pregrasp before the first lower target;
- no lower waypoint is sent when pregrasp convergence times out;
- the 100 mm approach is divided into consecutive segments no longer than
  5 mm and the duplicate start point is not resent as a descent step;
- the next lower waypoint is withheld until feedback confirms the current one;
- X/Y deviation during descent commands a hold at the observed pose and blocks
  all later waypoints and gripper close;
- Z timeout has the same hold-and-block behavior;
- the final vertical verification uses 1.5 mm X/Y and 3 mm Z limits and cannot
  fall back to the global 18 mm acceptance;
- `GRASP_DEBUG_STOP_AT_PREGRASP` occurs after verified reanchoring;
- non-vertical plans retain the existing `move_linear()` sequence and global
  final-approach tolerance;
- an elevated return pose is not mistaken for the initial monitored approach;
- a vertical approach failure is not treated as a retryable MoveIt pregrasp
  reachability failure.

Run the focused executor/config tests, then the related selector, planner,
trajectory, executor, and evaluation regression suite. Rebuild
`my_course_pkg` in the active Docker-mounted workspace after tests pass.

## Live Validation

Rebuild the active `/home/ws`, restart the full simulator launch, reset the
scene, and generate fresh perception for each trial.

First use `GRASP_DEBUG_STOP_AFTER_CLOSE=1` to inspect the critical event without
lifting or releasing. A qualifying close-only run must show:

- live-bounds X/Y centering remains active;
- pregrasp convergence passes before descent;
- every descent waypoint passes its X/Y and Z gate;
- neither finger lands on the brick's top surface;
- the brick is between the fingers after close.

Then use `GRASP_DEBUG_STOP_AFTER_LIFT=1`. A qualifying bounded lift must show the
brick rises with the gripper and no neighboring object is contacted.

Per the project-wide experiment policy, require two consecutive independent
qualifying bounded lifts. Each run starts from a restarted full launch, reset
scene, and fresh perception result. Only then run an ordinary lift-return trial.

## Acceptance Criteria

The work is complete when:

- focused and related regression tests pass;
- `my_course_pkg` builds in the active Docker-mounted workspace;
- logs prove that no lower target is issued before strict pregrasp alignment;
- no descent waypoint or gripper-close command follows an X/Y or Z gate failure;
- two consecutive independent bounded trials visibly capture and lift the foam
  brick without a finger striking its top surface;
- non-vertical regression behavior remains unchanged.

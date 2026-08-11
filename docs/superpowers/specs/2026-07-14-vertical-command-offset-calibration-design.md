# Vertical Command-Offset Calibration Design

## Goal

Make the strict `vertical` grasp approach reachable without weakening its
physical feedback limits. Learn the Cartesian controller's current XYZ
steady-state command bias at the safe pregrasp, apply the resulting command
offset to this run's descent commands, and continue judging safety against the
unmodified physical waypoints.

The calibration is temporary. It belongs to one attempted vertical grasp and
is discarded on success, failure, candidate change, or the next invocation.

## Evidence and Problem Statement

The first live run of the vertical feedback gate proved that the new code and
safety path were active:

- live-bounds centering produced the intended foam-brick target;
- MoveIt reached the collision-aware pregrasp;
- the executor entered the strict pregrasp gate;
- the gate issued a hold and raised before any descent or gripper close.

The nominal pregrasp target was approximately:

```text
[-0.4991, -0.9387, 1.0393] m
```

After 15 repeated nominal commands, feedback settled near:

```text
[-0.4990, -0.9317, 1.0232] m
```

Actual minus target was therefore approximately
`[+0.1, +7.0, -16.1] mm`. The strict gate correctly rejected the resulting
7.0 mm X/Y error and 16.1 mm Z error, but reissuing an identical target could
not remove the controller's repeatable steady-state mapping error.

Earlier controlled trials also showed that raising the commanded foam-brick Z
target by 20 mm raised the observed Z by approximately 19.9 mm while preserving
the repeatable lateral bias. That supports a bounded command-space correction
at safe hover instead of relaxing physical acceptance tolerances.

## Considered Approaches

### Per-run safe-hover command calibration

Measure the residual at pregrasp, accumulate the opposite residual into a
temporary command offset, and freeze the learned offset for the descent. This
adapts to the current controller state while preserving the nominal physical
trajectory and is the selected approach.

### Hard-coded foam-brick offset

Always add the observed `[-0.1, -7.0, +16.1] mm` correction. This is simple but
couples execution to one run, one controller state, and one object. It can
become stale and is rejected.

### Controller-wide tuning

Change the low-level Cartesian controller, pose source, or gains. This may be a
useful later systems fix, but it affects every motion profile and is much wider
than the confirmed vertical-grasp problem. It is outside this change.

## Selected Calibration Algorithm

Calibration operates on a nominal pregrasp pose and never changes its stored
plan representation.

1. Start with `command_offset_xyz = [0, 0, 0]`.
2. Read three live TCP samples, separated by the existing final-approach command
   period, and compute a component-wise median XYZ translation. Retain the
   newest raw full 6D sample separately for any hold command; do not average or
   take a median of Euler angles.
3. Compute the physical residual:

   ```text
   residual_xyz = nominal_pregrasp_xyz - observed_median_xyz
   ```

4. If the median feedback is within the existing 1.5 mm world-XY and 3 mm Z
   gates, return the current offset without another command.
5. Otherwise accumulate the residual:

   ```text
   proposed_offset_xyz = command_offset_xyz + residual_xyz
   ```

6. Fail closed before sending the compensated command if the Euclidean norm of
   `proposed_offset_xyz` exceeds 30 mm.
7. Copy the nominal pregrasp pose, add `proposed_offset_xyz` to XYZ only, and
   send that compensated pose. Preserve the nominal orientation exactly.
8. Wait 0.4 seconds for the controller response, then collect the next three
   median samples and repeat.
9. Allow at most three compensated commands. After the third command, collect
   and evaluate one final median sample set. If it still fails the physical
   gate, hold the observed pose and abort.

The recorded live residual would produce an initial command offset near
`[-0.1, -7.0, +16.1] mm`; that value is test evidence, not a constant.

## Divergence Rule

Let `residual_norm` be the 3D norm of the median physical residual. Record the
norm immediately before each compensated command. After settling and sampling,
fail as divergent if the new residual norm is greater than the prior norm plus
the larger configured position tolerance:

```text
divergence_margin_m = max(
    VERTICAL_APPROACH_XY_TOLERANCE_M,
    VERTICAL_APPROACH_Z_TOLERANCE_M,
)
```

With current defaults, the margin is 3 mm. This permits small measurement noise
but prevents continuing when compensation makes the physical error materially
worse. The 30 mm total-offset cap and three-command limit remain independent
failure gates.

## Calibration Result and Lifetime

Represent a successful result explicitly with a small value object containing:

- the final median actual XYZ;
- `command_offset_xyz`;
- the number of compensated commands.

`execute_plan()` keeps this result in a local variable initialized to zero for
each plan execution. It passes the offset explicitly into the initial vertical
approach method. The executor must not store it in module configuration, class
state, the plan, an object-specific table, or a file.

A failed calibration raises immediately. `execute_first_reachable_plan()` must
not treat it as a retryable MoveIt pregrasp reachability failure.

## Compensated Descent Data Flow

The trajectory planner and interpolation continue to generate the same nominal
pregrasp and 5 mm nominal descent waypoints.

For each nominal waypoint:

1. Before issuing the lower waypoint, compare current feedback with the nominal
   waypoint's X/Y as the existing gate does.
2. Build a separate command pose:

   ```text
   command_waypoint_xyz = nominal_waypoint_xyz + command_offset_xyz
   ```

3. Preserve the nominal waypoint orientation.
4. Send exactly one compensated command waypoint.
5. Poll feedback and compare it only with the nominal physical waypoint.
6. Require the existing 1.5 mm X/Y and 3 mm Z gates before issuing the next
   lower waypoint.

The offset is frozen once descent starts. No residual accumulation, command
offset update, or lateral correction is allowed beside the object. Lateral
deviation or Z timeout commands a hold at the observed pose and aborts before
the next lower waypoint or gripper close.

The final vertical verification also compares feedback with the nominal final
grasp pose. It never accepts proximity to the compensated command as success.

## Configuration

Add validated settings:

```text
GRASP_VERTICAL_REANCHOR_MAX_OFFSET_M=0.03
GRASP_VERTICAL_REANCHOR_MAX_ITERATIONS=3
GRASP_VERTICAL_REANCHOR_SETTLE_SEC=0.4
GRASP_VERTICAL_REANCHOR_SAMPLE_COUNT=3
```

Validation rules are:

- maximum offset and settle duration are finite and strictly positive;
- maximum iterations is an integer at least one;
- sample count is an odd integer at least three.

Sampling uses the existing
`GRASP_FINAL_APPROACH_COMMAND_PERIOD_SEC` between samples. The strict X/Y and Z
tolerances and 5 mm descent spacing remain unchanged.

## Component Boundaries

### `config.py`

Owns parsing, defaults, and validation for the four calibration settings.

### `executor.py`

Owns median sampling, residual and divergence calculations, bounded calibration,
the temporary calibration result, compensated command-pose construction, and
all feedback gating.

The pregrasp calibration method returns the explicit result. The vertical
approach and individual-waypoint methods accept `command_offset_xyz` as an
argument. This makes the compensation dependency visible and prevents stale
state from leaking between runs.

### Unchanged components

The trajectory planner, grasp selector, FoundationPose conversion, live-bounds
centering, grasp candidate orientation, gripper targets, lift, return, release,
and non-vertical executor paths remain unchanged.

## Failure Handling and Diagnostics

Calibration fails closed and commands a hold at the newest valid raw full 6D
pose, not a synthesized median orientation, when any of these conditions occurs:

- a target, feedback sample, median, residual, proposed offset, or command pose
  contains a non-finite value;
- the proposed total command offset exceeds 30 mm;
- residual divergence exceeds the defined 3 mm default margin;
- three compensated commands are exhausted without satisfying the physical
  gate;
- live TCP feedback cannot be obtained.

The existing descent lateral-deviation and Z-timeout behavior remains in force.

Each calibration iteration logs:

- iteration index and maximum;
- nominal target XYZ;
- compensated command XYZ;
- the three observed sample translations and their median;
- residual XYZ and norm;
- accumulated command offset XYZ and norm;
- physical X/Y and Z errors and their tolerances.

Success logs the frozen offset. Descent logs both nominal and compensated XYZ,
while gate success and failure continue to report errors relative to nominal.

## Scope Boundaries

This change deliberately does not:

- hard-code the recorded foam-brick correction;
- persist or reuse an offset across plans, candidates, objects, or processes;
- update the offset after descent begins;
- relax the 1.5 mm X/Y, 3 mm Z, or 5 mm waypoint settings;
- change grasp orientation, object bounds centering, or grasp Z selection;
- change non-vertical motion or the global 18 mm gate;
- tune the low-level arm controller;
- add object-lift perception or alter success reporting.

## Tests

Configuration tests cover defaults, overrides, malformed values, zero,
negative, non-finite values, invalid integers, and even or too-small sample
counts.

Executor tests must prove:

- an already aligned pregrasp returns a zero offset without a compensated
  command;
- the recorded `[+0.1,+7.0,-16.1] mm` actual-minus-target bias produces an
  approximately `[-0.1,-7.0,+16.1] mm` command offset and then converges;
- a second residual is added to the first offset rather than replacing it;
- component-wise median sampling rejects one large outlier;
- orientation is unchanged in every compensated command;
- an offset above 30 mm holds and fails before sending the unsafe command;
- material residual divergence holds and fails immediately;
- iteration exhaustion holds and blocks descent and gripper close;
- each compensated descent command equals nominal XYZ plus the frozen offset;
- descent and final acceptance errors are calculated against nominal waypoints,
  not compensated commands;
- the offset is unchanged during descent;
- every new plan starts with zero offset and failed plans do not leak state;
- elevated return motion is not compensated;
- non-vertical plan behavior and the global final gate remain unchanged.

Run focused config/executor tests, the related selector/planner/trajectory/
executor/evaluation suite, and a `my_course_pkg` symlink rebuild in the active
Docker-mounted workspace. Confirm the four modified runtime/test files match
between workspaces apart from line endings.

## Live Validation

After rebuilding, restart the complete simulator launch, verify unique arm and
gripper servers, reset the scene, and run fresh foam-brick perception.

First run with `GRASP_DEBUG_STOP_AFTER_CLOSE=1`. A qualifying close-only trial
must show:

- bounds centering still targets the live foam-brick center;
- calibration learns a finite offset no larger than 30 mm;
- median physical pregrasp feedback passes the strict X/Y and Z gate;
- every descent log distinguishes nominal and compensated waypoints;
- every waypoint passes feedback relative to nominal;
- neither finger lands on the brick's top surface;
- the brick is between the fingers after close.

Only after that passes, use `GRASP_DEBUG_STOP_AFTER_LIFT=1` for two consecutive
independent qualifying runs. Restart the full launch, reset, and run fresh
perception for each trial. The brick must rise with the gripper and avoid all
neighboring objects. Only then run an ordinary lift-return trial.

## Acceptance Criteria

The work is complete when:

- focused and related regression tests pass;
- `my_course_pkg` builds and installed defaults match the design;
- the offset is learned per run, bounded, frozen for descent, and discarded;
- physical gates remain relative to nominal waypoints and fail closed;
- one bounded close-only trial shows both fingers clear the top surface;
- two consecutive independent bounded lift trials visibly lift the foam brick;
- non-vertical behavior remains unchanged.

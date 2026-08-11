# Project-Wide Two-Run Experiment Policy Design

## Status

Approved by the user on 2026-07-14. This document defines validation policy;
it does not by itself authorize robot motion.

## Decision

Every current and future formal experiment, staged validation, stability
qualification, category rollout, and randomized-position qualification in this
project requires two consecutive independent successes.

Two successes are sufficient for qualification. Additional runs remain useful
as optional diagnostics, but they are not a release gate. Any failed formal run
resets the consecutive-success count to zero.

This policy supersedes older normative references that require three runs.
Historical logs and descriptions of experiments that actually used three runs
remain factual and must not be rewritten.

## Scope

The two-run rule applies to:

- perception-only and selector-only qualification stages;
- MoveIt plan-only qualification;
- staged pregrasp, final-grasp, close, lift/hold, place, release, and retreat
  validation;
- complete pick-place closed-loop qualification;
- category representatives and remaining objects within each category;
- fixed, discrete, yaw-randomized, continuous-position, and fully randomized
  scene qualification.

An independent run starts from the reset or initialization required by its
stage and uses fresh inputs. For perception-dependent grasp experiments, that
means a simulator reset followed by a fresh pipeline result. Reusing stale
perception output does not count as an independent run.

## Safety Invariants

Only the number of required consecutive successes changes. The following
requirements remain unchanged:

- collision, approach-corridor, table, base, and placement clearances;
- finite and fresh scene-bounds data, including fail-closed behavior;
- object geometry, gripper-opening, and finger-clearance gates;
- MoveIt reachability and candidate-fallback behavior;
- TCP position and orientation convergence limits;
- gripper close/open acceptance and release validation;
- lift height, active hold duration, post-release observation, and object
  retention checks;
- the rule that each experiment changes at most one primary variable;
- the requirement to save failure evidence and restart qualification after a
  failed formal run.

No object-specific bypass, collision-margin reduction, stale-data reuse, or
manual target/candidate override may be introduced to obtain the two required
successes.

## Implementation Design

### Runtime evaluator

Change the generic evaluator's required consecutive-success constant from
three to two. Keep its streak semantics: a failure breaks the streak, one
success is insufficient, and two consecutive successes qualify.

Explicitly requesting more than two diagnostic trials remains allowed. Such a
request does not increase the qualification minimum unless a future policy
change explicitly says so.

### Tests

Update focused evaluator tests so they prove:

1. one successful run does not qualify;
2. two consecutive successful runs qualify;
3. a failure between successes resets the streak;
4. later consecutive successes can form a new qualifying streak;
5. result summaries state the two-success requirement correctly.

Run the related grasp/evaluator regression suite and build
`my_course_pkg`. No motion is part of this verification.

### Documentation

Update active normative specifications, plans, current handoff state, and
useful commands that still define three runs as a requirement. Do not perform
a blind numeric replacement: dimensions, 3D error descriptions, trial IDs,
historical measurements, and unrelated counts must remain unchanged.

Where an older committed design still says three runs, either update the
normative text or add an explicit reference to this superseding policy. Keep
historical `Recent Work` entries factual.

## Current Apple Transition

The two completed independent Apple A3 plan-only trials satisfy the new `2/2`
qualification requirement. Both used fresh Apple perception, retained all
eight candidates with `method=exact_bounds_box_2p5d`, accepted pregrasp
candidate 1 on the first MoveIt plan-only attempt, restored
`planonly=False`, and executed no trajectory or gripper command.

Apple A3 is therefore complete under the new policy. This removes the A3
repeat-count gate, but it does not by itself unlock Phase B motion.

## Phase B Readiness Gate

The project previously relaxed the `>=18 Hz` state/control health requirement
only for static A3 plan-only validation. Real arm motion still requires that
gate. The latest accepted single-camera measurement is about `3.43 Hz`, so the
current active configuration is not yet qualified for Phase B.

Before any `grasp_demo` run, the active launch must demonstrate the approved
`>=18 Hz` control-loop and `/joint_states` health gate with no stale simulator
processes. The fresh `/scene_clearance_bounds` safety contract must remain
available and valid; cadence recovery must not disable or bypass it.

If the active configuration remains below the gate, stop before motion and use
a separately reviewed cadence-recovery design. Do not use the known-unsafe
640x480 configuration while perception intrinsics remain hard-coded for
1280x720, and do not lower the health threshold merely to enter Phase B.

## Phase B Design: Move To Pregrasp And Stop

Phase B performs real arm motion, so each of its two formal runs must use the
existing pregrasp debug stop and proceed independently. These steps are
authorized only after the Phase B readiness gate passes:

1. reset the simulator and verify the robot is at its initial pose;
2. verify the gripper is visibly open at reset;
3. run a fresh pipeline that naturally selects Apple, without candidate or
   mask overrides;
4. run `grasp_plan_only` against that fresh perception result and require the
   exact-bounds corridor, MoveIt reachability, and `planonly=False` cleanup
   gates to pass;
5. review the Apple mask and fresh FoundationPose output, then enable only
   `GRASP_DEBUG_STOP_AT_PREGRASP=1` and clear all later debug-stop
   variables;
6. run `grasp_demo` and allow only the MoveIt motion to pregrasp;
7. inspect the held pregrasp pose, then press Ctrl-C to stop the process before
   starting the next independent run;
8. reset and repeat the complete sequence for the second formal run.

The existing executor path skips the pre-approach gripper command when the
pregrasp debug stop is active, executes `move_to_pre_grasp`, records
`at_pregrasp`, and returns before Cartesian approach, close, lift, drop, or
release. Because it skips the open command, the reset-state open-gripper check
is mandatory.

Each Phase B run passes only when:

- the target is Apple and all perception/scene data belong to the current
  reset;
- at least one exact-corridor-safe candidate is MoveIt-reachable;
- the actual TCP reaches the selected pregrasp target within the existing
  planning/control tolerance;
- Apple lies below the finger centerline with a clear vertical approach path;
- the gripper remains open;
- no Cartesian final approach, gripper, lift, placement, or release command is
  executed;
- Apple and all non-target scene objects remain stationary apart from passive
  simulator settling.

Any violation ends the run, saves the logs, and resets the Phase B success
count to zero. Do not advance to Phase C until two consecutive Phase B runs
pass.

## Completion Criteria

This policy change is implemented when:

- the generic evaluator and focused tests use two consecutive successes;
- active normative documentation consistently identifies two as the required
  count;
- related tests and the package build pass;
- the handoff records Apple A3 as complete at `2/2` and identifies Phase B as
  the next action;
- no safety threshold, motion path, selector behavior, scene layout, or
  simulator physics setting changes as part of the policy update.

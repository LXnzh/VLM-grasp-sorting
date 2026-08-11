# Apple Close-Minimum Runtime Override Design

**Date:** 2026-07-14  
**Status:** User-approved design

## Goal

Allow the complete Apple pick-transfer-place experiment to continue after a
visually correct grasp when the simulated gripper reports a stalled contact at
position `0.047`, just below the current generic minimum useful close position
of `0.050`.

## Evidence

The first complete Apple trial reached the selected final grasp and wrapped the
fingers around the Apple. The close action then reported:

- requested position: `0.790`;
- actual position: `0.0470588`;
- effort: `139.7`;
- `stalled=True` and `reached_goal=False`.

The existing gripper adapter rejects a stalled close only when its actual
position is below `GRIPPER_CLOSE_MIN_POSITION`. The default is `0.050`, so the
otherwise useful Apple contact was rejected by about `0.003` and execution
stopped before the post-close hold and lift.

## Approved Behavior

Set the existing runtime configuration variable only for the Apple experiment:

```bash
export GRASP_GRIPPER_CLOSE_MIN_POSITION=0.04
```

This keeps the repository default at `0.050`. A stalled close at the observed
`0.047` can be accepted, while a stalled close below `0.040` still fails.

The override does not change:

- target selection, masking, FoundationPose, grasp generation, or collision
  filtering;
- MoveIt reachability or Cartesian final-approach convergence checks;
- the gripper action result fields or the requirement for the existing close
  acceptance expression to succeed;
- open-gripper and release validation;
- lift, transfer, placement, retreat, or return-home behavior.

No effort threshold is added. The high effort is supporting evidence for this
specific observed contact, not a new acceptance condition.

## Trial Workflow

The stopped trial does not qualify as a complete success. Do not resume from
the held Apple pose because the camera and object state have changed.

Each retry must:

1. set `GRASP_GRIPPER_CLOSE_MIN_POSITION=0.04`;
2. clear every `GRASP_DEBUG_STOP_*` variable;
3. reset the simulator;
4. run a fresh Apple pipeline;
5. success-chain the pipeline to the full `grasp_demo` command;
6. allow the program to continue through close, lift, transfer, placement,
   release, retreat, and return home.

The full experiment qualifies after two independent successful runs under the
project-wide two-run policy. A run passes when it naturally targets Apple,
completes the return-home step, and prints `Pick and place completed.` without
a grasp, gripper, motion, placement, release, or return-home failure.

## Failure And Rollback

If the close again reports an actual position below `0.040`, or any existing
runtime check fails, save the log and stop that run. Diagnose the new evidence
instead of lowering the threshold again automatically.

Remove the experiment-only behavior with:

```bash
unset GRASP_GRIPPER_CLOSE_MIN_POSITION
```

This immediately restores the repository default of `0.050` for newly started
processes.

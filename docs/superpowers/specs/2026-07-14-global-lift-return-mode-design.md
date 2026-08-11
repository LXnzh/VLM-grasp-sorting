# Global Lift-Return Mode Design

**Date:** 2026-07-14
**Status:** Approved in conversation

## Goal

Make the default grasp experiment evaluate stable acquisition and lifting only:
grasp the selected object, lift it vertically by `0.20 m`, hold it for `5 s`,
then return it to the original grasp location and release it. The default path
must not search for a separate placement location or perform a horizontal
transfer.

## Execution Modes

Add an explicit execution mode with two valid values:

- `lift_return` is the new default for every object category.
- `safe_place` preserves the existing obstacle-aware remote-placement workflow
  as an opt-in compatibility mode.

The mode is configured by `GRASP_EXECUTION_MODE`, whose default is
`lift_return`, and selected once when grasp configuration is loaded. Invalid
values fail immediately with a clear configuration error. Existing
`GRASP_PLACE_*` settings are consulted only by explicit `safe_place` mode.

## Default `lift_return` Sequence

For each grasp candidate, construct this sequence:

1. Open the gripper.
2. Use MoveIt to reach the pre-grasp pose.
3. Approach the grasp pose vertically.
4. Close the gripper and retain the existing result validation.
5. Retain the existing post-close settling hold.
6. Lift vertically from the grasp pose by `GRASP_LIFT_HEIGHT` (`0.20 m` by
   default).
7. Actively hold the lifted pose for `GRASP_LIFT_HOLD_SEC` (`5 s` by default).
8. Descend vertically to the original grasp pose while keeping the gripper
   closed.
9. Retain the existing one-second pre-release hold.
10. Open the gripper and retain the existing release validation.
11. Retain the existing post-release settling hold.
12. Retreat vertically to the original pre-grasp pose.

This sequence contains no safe-place search, `transfer_to_drop_high`, separate
drop pose, or horizontal object motion.

## Planner Integration

`pick_place_planner.py` branches on the execution mode before selecting a safe
placement point. In `lift_return` mode it must not call `_select_safe_place_xy`
and must not require scene data solely for placement. Scene-clearance data and
the existing exact-bounds checks remain active where they are required for the
pre-grasp-to-grasp approach.

`trajectory_planner.py` builds a dedicated lift-return step list rather than
reusing the remote-placement sequence with identical grasp and drop poses.
This avoids creating a no-op MoveIt transfer and removes the failure path that
currently occurs at `transfer_to_drop_high`.

The existing `safe_place` branch and its placement parameters remain available
without changing their semantics.

## Failure Behavior

All existing fail-closed behavior remains:

- a failed pre-grasp or Cartesian motion stops execution;
- an unacceptable gripper result stops execution;
- a failed lift, return descent, release, or retreat stops execution and reports
  the exact step name;
- no Cartesian fallback is added for a failed MoveIt pre-grasp motion.

There is no fallback from `lift_return` to remote placement.

## Verification

Focused tests must prove that:

- the default mode is `lift_return`;
- its lift displacement is exactly the configured `0.20 m` and its lifted hold
  is exactly the configured `5 s`;
- descent returns to the original grasp pose and retreat returns to the original
  pre-grasp pose;
- the default step list contains neither `transfer_to_drop_high` nor any safe
  placement selection;
- approach-clearance behavior remains enabled where required;
- explicit `safe_place` mode retains the existing remote-placement sequence;
- invalid execution modes fail clearly.

Run focused planner, trajectory, and executor tests, then rebuild
`my_course_pkg`. Live qualification remains two independent complete successes
per object category.

## Out of Scope

- Changing grasp candidate generation or object-specific profiles.
- Changing the `0.20 m` lift height or `5 s` hold defaults.
- Relaxing gripper acceptance, approach-clearance, or motion-convergence checks.
- Adding a new placement-reachability planner for the legacy `safe_place` mode.

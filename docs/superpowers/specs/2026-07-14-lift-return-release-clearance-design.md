# Lift-Return Release-Clearance Design

**Date:** 2026-07-14
**Status:** Approved in conversation

## Goal

Keep the experiment focused on successful acquisition, a `0.20 m` lift, and a
five-second hold, while preventing the later return/release sequence from
failing when the held object contacts the table before the TCP reaches the
original planned grasp pose.

The post-hold behavior should remain deliberately small and easy to merge with
a teammate's future downstream workflow.

## Observed Failure

The tomato trial completed grasp, lift, and the five-second hold. During
`return_to_grasp`, the can reached the table while the gripper was still
closed. Table contact left the TCP `0.0161 m` from the original planned grasp
pose, exceeding the existing `0.013 m` convergence tolerance. The executor
therefore raised `FinalApproachConvergenceError` before release and retreat.

The existing convergence tolerance must not be widened globally because it
also protects the initial pregrasp-to-grasp approach.

## Selected Behavior

In `lift_return` mode, return to a release pose directly above the original
grasp TCP instead of returning all the way to the original grasp TCP:

```text
release_pose = grasp_pose
release_pose.world_z += GRASP_RETURN_RELEASE_CLEARANCE_M
```

`GRASP_RETURN_RELEASE_CLEARANCE_M` defaults to `0.02 m`. Negative values are
invalid. The offset is along world `+Z`, matching the vertical lift/return
workflow and the horizontal simulation table.

The existing step names and ordering remain compatible:

1. Grasp the object using the existing strict final-approach convergence gate.
2. Lift by the configured lift height.
3. Hold for the configured five seconds.
4. Descend to the release pose `0.02 m` above the original grasp TCP.
5. Perform the existing pre-release hold.
6. Open the gripper.
7. Perform the existing post-release hold.
8. Retreat to the original pregrasp pose.

The object may fall approximately `0.02 m` to the table when released. Actual
drop distance can differ slightly if the object moved within the fingers.

## Code Boundaries

- `config.py` adds the single runtime setting
  `GRASP_RETURN_RELEASE_CLEARANCE_M`, defaulting to `0.02`, and rejects a
  negative value while loading grasp configuration.
- `trajectory_planner.py` applies the setting only while constructing the
  `lift_return` plan. Its `return_to_grasp`, release holds, and retreat start
  use the elevated release pose.
- The initial `approach_grasp` target and
  `GRASP_FINAL_APPROACH_POSITION_TOLERANCE_M=0.013` remain unchanged.
- Explicit `safe_place` mode remains byte-for-byte behaviorally unchanged.
- No contact detector, force threshold, table-height estimator, or dynamic
  release state machine is introduced.

Because the elevated return target is no longer equal to
`plan.grasp_pose_6d`, the executor's existing final-grasp convergence hook is
not applied to the return step. The normal Cartesian command path still sends
the complete elevated return trajectory. This is intentional for the minimal
post-hold workflow; downstream contact-aware placement remains the teammate's
scope.

## Failure Behavior

- Initial approach non-convergence continues to stop before gripper close.
- Gripper close and release result validation remain unchanged.
- Motion or process errors outside the removed original-pose return check
  continue to propagate normally.
- The release clearance avoids intentionally commanding the held object into
  the table; it is not treated as collision detection for arbitrary obstacles.

## Verification

Focused tests must prove that:

- the default release clearance is exactly `0.02 m`;
- negative release clearance is rejected clearly;
- a `lift_return` plan returns to a pose whose world Z is the grasp-pose Z plus
  the configured clearance;
- `return_to_grasp` ends at the elevated release pose, both release holds use
  it, and `retreat_after_release` starts from it;
- the initial grasp pose and strict initial convergence behavior are unchanged;
- a zero override preserves the legacy return pose for explicit debugging;
- `safe_place` plans are unchanged;
- the existing lift height, five-second hold, and step ordering remain intact.

Run the focused configuration, trajectory-planner, and executor tests, then
rebuild `my_course_pkg`. A live tomato smoke should show successful grasp,
`0.20 m` lift, five-second hold, descent to the elevated release pose, gripper
open, and retreat without `FinalApproachConvergenceError`.

## Out of Scope

- Contact-aware placement or force/torque sensing.
- Exact table-touch release.
- Changing grasp selection, approach clearance, lift height, or hold duration.
- Redesigning the teammate-owned downstream placement workflow.
- Relaxing the global initial final-approach tolerance.

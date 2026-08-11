# Safe Place And Release Design

## Goal

After a successful tomato-can grasp, the robot must lift the object by at
least 20 cm, hold that lifted pose for at least 5 seconds, then move to a safe
table location and release only if the gripper actually opens. The post-grasp
sequence must be predictable and fail closed; it must not continue into retreat
or home motion after an unhealthy release command.

## Current failure

The grasp and lift can succeed, but the existing place stage still descends to
a fixed `DROP_Z=0.35`. That height is not derived from the table, the grasped
object, or the current successful grasp pose. In the same run, the release
command asked for an open position but returned stalled with
`reached_goal=False`; the current gripper API collapses that result to `True`,
so the executor cannot distinguish a valid close-on-object stall from a failed
open command.

## Design

1. Replace boolean gripper feedback with a structured result containing the
   requested target, actual gripper position, effort, action status, stalled
   state, and `reached_goal`.
2. Keep close-on-object behavior permissive enough to accept a stall, because
   stalled fingers can mean the can is held.
3. Treat open/release commands strictly. A release command must reach the open
   goal or report an actual position close to the requested open position. A
   stalled open command is a failure, not success.
4. Make `hold_after_lift` active: keep streaming the lifted TCP pose during the
   hold window instead of sleeping passively.
5. Compute the place/release TCP height from the successful grasp pose, and
   compute retreat height from the configured lift height. Do not use the old
   fixed `DROP_Z` as the release height.
6. Select a safe place XY from the scene description. The selected point must
   be inside a configurable table region and outside conservative clearance
   disks around all non-target scene objects. If no safe point exists, refuse to
   run the place sequence.
7. Add debug stops after lift and before release so live trials can validate
   lift stability and the selected place pose before opening the gripper.

## Testing

Focused tests should cover:

- gripper result fields are preserved for executor decisions;
- close can accept a stalled result while release rejects a stalled-not-open
  result;
- lifted hold uses active pose holding for the configured 5 seconds;
- drop/release height is derived from grasp height and retreat height uses
  lift height;
- safe place selection avoids all non-target objects and fails when the table
  region has no valid placement candidate.

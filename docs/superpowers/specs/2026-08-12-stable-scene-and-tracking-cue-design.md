# Stable Scene Placement and Tracking Cue Design

## Context

The GUI product launches the simulator with `scene_mode=random`, but this mode
is intentionally stratified rather than uniformly random. It selects one
object from each of five ordered grasp categories and then one additional
object from the remaining pool. With the current category configuration, the
tomato soup can, banana, and hammer are always present; the other three
identities vary. The six selected identities are assigned to six fixed sorting
slots. `/reset_sim` restores the already compiled scene and does not resample
its identities.

This selection and layout are accepted product behavior and must not change.
Two usability defects remain:

1. Mesh Z placement uses the visual mesh bottom. The hammer's collision mesh
   extends about 0.33 mm below its visual mesh, so its initial collision shape
   intersects the tabletop and can be expelled by the contact solver.
2. The GUI's `TARGET_LOCKED` guidance is replaced almost immediately by a
   stable-state message. The following `PBVS_OBSERVING_FOR_MOTION` event is not
   mapped to operator guidance, and the current three-second observation
   period is too short for a user to switch to the MuJoCo window and drag the
   selected object.

## Goals

- Preserve the existing stratified six-object sampling rule and fixed slots.
- Spawn the hammer, and every other mesh object, without collision penetration
  into the tabletop.
- Give the operator an obvious ten-second opportunity to start target motion.
- Keep the "may move" and "must remain still" states unambiguous.
- Preserve all stable-grasp, sorting, VLM, SAM2, tracking, PBVS, and
  FoundationPose behavior outside this narrow change.

## Non-goals

- Uniformly sampling six objects from the sixteen-object catalog.
- Resampling objects when `/reset_sim` is called.
- Changing object slots, category membership, bin placement, or supported
  identities.
- Adding a hammer-specific pose offset or physics exception.
- Changing grasp candidates, planning, guarded execution, or placement logic.
- Adding a new configuration layer for scene placement.

## Scene placement design

Mesh placement keeps two distinct geometric responsibilities:

- Visual-mesh XY centroid: continue centering the visible object in its
  assigned slot.
- Collision-mesh Z minimum: place the physical collision bottom above the
  table, because this is the geometry used by MuJoCo contacts.

`_compute_mesh_placement_info()` will load every validated collision mesh in
addition to the visual mesh. It will retain the visual centroid fields and
return the minimum transformed collision Z. The scene builder will calculate
the body Z from that collision minimum and a fixed 0.5 mm positive clearance.
The small clearance prevents initial intersection without creating a visible
drop or a per-object tuning value.

The same transformation is applied to both visual and collision vertices when
an object has a configured orientation. Missing, empty, or non-finite
collision geometry remains a hard scene-construction failure through the
existing YCB asset validation path.

## Tracking-window and GUI design

The PBVS node's `pbvs_motion_observation_s` default changes from 3.0 seconds to
10.0 seconds. The window still starts only after target initialization has
completed. A static target may proceed automatically when the window expires;
movement is not required. If movement begins during the window, PBVS continues
following until the target and camera both satisfy the existing continuous
stop criteria.

The GUI will add a full-width operator-guidance banner above the complete ROS
log. This is separate from the compact `Robot state` text so the actionable
instruction is visible even when detailed state messages change rapidly.

The banner states are:

1. Before target lock: `KEEP STILL` while VLM and SAM2 select the instance.
2. On `TARGET_LOCKED` or `PBVS_OBSERVING_FOR_MOTION`:
   `TRACKING READY — 10 seconds: select the target in MuJoCo and
   Ctrl+Shift+right-drag it now.`
3. On `PBVS_FOLLOW_ACTIVE` or `PBVS_COMMAND`:
   `PBVS FOLLOWING — continue moving, or release the target and keep it still.`
4. On `PBVS_WAITING_FOR_CONTINUOUS_STOP`:
   `TARGET RELEASED — keep it still while the stop window fills.`
5. On `PBVS_RELATIVE_POSE_STABLE`, `FOUNDATIONPOSE_REQUEST`, planning, or
   execution:
   `KEEP STILL — pose estimation/grasp has started.`
6. On failure, tracking loss, or completion: show the corresponding terminal
   outcome.

While the drag window is active, a `PBVS_MOTION_GATE: state=STABLE` message
must not replace the `TRACKING READY` guidance. The GUI clears the drag-window
state only when following/stop/freeze/failure/completion provides a more
specific state. The underlying ROS log remains complete and unchanged.

## Error handling and safety

- Collision geometry errors stop scene construction instead of falling back
  to the visual bottom.
- Dragging remains forbidden before target lock and after relative-pose freeze.
- The ten-second change only delays a static grasp; it does not weaken motion,
  tracking-loss, stable-stop, pose, planning, or guarded-execution checks.
- No automatic object motion or scripted perturbation is introduced.

## Verification

Automated checks will cover:

- existing random-mode tests still select one object per ordered category plus
  one unique remaining object, for exactly six identities;
- mesh placement uses the visual XY centroid and collision Z minimum under
  identity and rotated orientations;
- the generated hammer collision bound begins at least 0.5 mm above the table;
- a short headless MuJoCo settling check shows no hammer launch, using 5 mm as
  the maximum accepted upward or horizontal displacement from the expected
  settling neighborhood;
- the PBVS default observation duration is 10 seconds;
- GUI state tests prove `PBVS_OBSERVING_FOR_MOTION` displays the full-width
  tracking instruction and a stable gate cannot overwrite it prematurely;
- follow, stop, freeze, failure, and completion messages replace the banner
  correctly;
- complete simulator and `my_course_pkg` functional suites, static checks, and
  package/workspace builds remain green.

Manual acceptance uses one fresh GUI scene:

1. Confirm the hammer settles in place without visibly jumping away.
2. Start a banana task and keep the target still through VLM/SAM2.
3. Confirm the full-width ten-second `TRACKING READY` banner appears.
4. Use `Ctrl+Shift+right-drag` to move the banana across the tabletop and
   observe `PBVS FOLLOWING` plus camera/robot response.
5. Release the banana, keep it still, and verify FoundationPose, grasp, lift,
   classified placement, and return home complete.

The final saved GUI log and outcome are recorded in `HANDOFF.md`.

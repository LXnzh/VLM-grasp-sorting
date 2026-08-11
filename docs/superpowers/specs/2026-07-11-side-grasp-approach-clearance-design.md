# Side Grasp Approach Clearance Design

## Goal

Reject side-grasp candidates whose final approach corridor would collide with
nearby non-target objects. This is a target-candidate safety filter, not a full
MoveIt planning-scene replacement.

The immediate problem is that `move_to_pre_grasp` can succeed, but the later
servo/interpolated `pregrasp -> final grasp` approach is not checked against
scene objects. A side-grasp candidate can therefore be geometrically good for
the tomato can while still sweeping through any other object that happens to
spawn nearby. Banana is only the current example, not a special case.

## Scope

- Apply only to side-grasp candidates.
- Check only the straight approach segment from `plan.pre_grasp_pose_6d` to
  `plan.grasp_pose_6d`.
- Use `/scene_description` `MarkerArray` data as the source for non-target
  object positions and marker sizes.
- Exclude the selected target object from obstacle checks.
- Fail safely: if clearance checking is enabled but all candidates are blocked,
  raise an error before arm motion.
- Keep perception, FoundationPose, final approach convergence, gripper logic,
  and lift verification unchanged.

## Options Considered

1. Approach-corridor candidate filter using scene markers.
   This is the selected option. It directly addresses side approaches that
   sweep through nearby objects and is small enough to test and iterate.
2. Full MoveIt planning-scene object insertion.
   More complete for `move_to_pre_grasp`, but larger and more fragile because
   marker geometry, TF, scene updates, and planner synchronization all become
   part of the live control loop.
3. Point-cloud occupancy checking.
   Most general, but too heavy for the current debugging stage and harder to
   explain when it rejects a candidate.

## Design

Add a side-grasp clearance filter in `plan_pick_place_candidates_from_perception`
after each candidate plan is created and after the existing world-Z sanity
filter.

The filter will:

1. Subscribe once to `/scene_description` and wait briefly for a `MarkerArray`.
2. Convert each marker into a conservative obstacle sphere in `base_link/world`
   coordinates.
3. Ignore markers whose name/text matches the selected target object.
4. For each side-grasp candidate, compute the shortest XY distance from every
   obstacle center to the pregrasp-final line segment.
5. Reject the candidate when:
   - the XY distance is below `obstacle_radius + corridor_radius + margin`, and
   - the obstacle vertical range overlaps the approach vertical range expanded
     by `vertical_margin`.

The corridor radius is a conservative approximation of the Robotiq fingers,
TCP frame uncertainty, and object motion during the approach. It intentionally
over-approximates rather than trying to be a precise mesh collision checker.

## Configuration

Add environment-backed settings:

- `GRASP_SIDE_CLEARANCE_ENABLED`, default `1`.
- `GRASP_SIDE_APPROACH_CORRIDOR_RADIUS_M`, default `0.08`.
- `GRASP_SIDE_APPROACH_CLEARANCE_MARGIN_M`, default `0.03`.
- `GRASP_SIDE_APPROACH_VERTICAL_MARGIN_M`, default `0.05`.
- `GRASP_SCENE_DESCRIPTION_TIMEOUT_SEC`, default `0.5`.

If the scene description is unavailable within the timeout, log a warning and
skip the filter for that run. This avoids blocking all grasping if the
simulation publisher is down, while still making the reason visible.

## Diagnostics

For each rejected candidate, print:

- candidate index;
- obstacle name;
- XY distance to the approach segment;
- required clearance;
- approach Z range and obstacle Z range.

After filtering, print a summary:

```text
Side-grasp approach clearance: checked=<n>, rejected=<m>, remaining=<k>
```

If all candidates are rejected, raise a `RuntimeError` explaining that no
side-grasp candidate clears nearby scene objects.

## Tests

Add focused planner tests with fake scene markers:

- Any non-target object near the pregrasp-final segment rejects that candidate.
- An obstacle far from the segment does not reject the candidate.
- The selected target marker is ignored.
- If every candidate is blocked, planning raises before execution.
- If scene data is missing, the planner logs/skips the filter rather than
  crashing.

## Live Verification

1. Reset the simulation and rerun `pipeline` for the tomato can.
2. Run a guarded close-only grasp with clearance enabled.
3. Confirm logs show candidate(s) near the banana rejected.
4. Confirm the selected candidate approaches from a side that does not sweep
   through the banana.
5. Confirm no close/full pick-place is attempted until final TCP convergence is
   separately healthy.

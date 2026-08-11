# Plan-only Grasp Candidate Verification Design

## Goal

Choose the first ranked stable grasp that MoveIt can plan as one collision-
checked path from the robot's current state through pre-grasp, grasp, and a
world-Z lift.  Candidate screening must not move the MuJoCo robot or command
the gripper.

## Scope

- Screen the first `GRASP_VERIFICATION_TOP_K` ranked candidates (default 20).
- For each candidate, construct these poses in the `world` frame:
  1. pre-grasp: offset `-GRASP_APPROACH_DIST` along the grasp frame's local
     `+Z` approach axis;
  2. grasp: the candidate grasp pose; and
  3. lift: same orientation and X/Y as grasp, with world `z + 0.20 m`.
- Send all three poses as one `arm/move_to_pose_path` request while
  `arm/set_planonly` is true.  A successful response means the complete
  approach-and-lift path is collision-free and reachable from the current
  robot state.
- Return the first passing candidate and a count of failures.  A controlled
  error is raised when no candidate passes.

## Architecture

`candidate_verifier.py` is pure Python.  It creates pre-grasp/lift poses and
iterates ranked candidates using an injected `plan_path` callable.  This makes
geometry and rejection behaviour testable without ROS2.

`ArmMotionExecutor` owns the ROS2 adapter.  Its new plan-only method enables
`set_planonly(True)`, converts the three 6D poses to `PoseStamped`, calls
`move_to_pose_path`, and always disables plan-only mode afterwards.  It does
not call the gripper or servo interfaces.

`pick_place_planner.py` supplies ranked candidates to the pure verifier, then
builds the normal executable pick plan from the selected pose.  The execution
plan lifts in world Z by 20 cm and holds for five seconds, matching the course
task.

## Data Flow

```text
stable grasp archives
  -> geometry ranking (top K)
  -> candidate verifier
  -> MoveIt plan-only path: pre-grasp -> grasp -> lift
  -> first feasible candidate
  -> executable pick plan
```

## Failure Handling and Metrics

Each rejected candidate records its ranked index and whether the path planner
rejected it.  The selection result exposes `candidates_tested` and
`rejected_candidates`.  The demo logs these values before any gripper command
or motion is executed.  A plan-only service failure is treated as an error,
not a feasible plan.

## Tests

- lift pose changes only world Z by exactly 0.20 m;
- verifier accepts the first candidate whose injected planner returns true;
- verifier skips failed candidates, observes the configured top-K bound, and
  reports the number tested/rejected;
- verifier raises a clear error if all candidates fail; and
- existing trajectory-plan test confirms a 20 cm world-Z lift and 5 second
  hold.

## Non-goals

This slice does not change perception, add object-category scoring, or prove
physical grasp success.  It only selects a collision-checked, kinematically
reachable candidate before executing the existing grasp sequence.

# Experiment Session DDS Graph Settle Implementation Plan

## Objective

Implement the approved bounded duplicate-settle policy in the experiment
session without changing process ownership, launch behavior, or downstream
motion stages.

## Task 1: Encode Temporal Graph-Gate Behavior in Tests

Modify `src/my_course_pkg/test/test_experiment_session.py`.

- Add deterministic ROS CLI response helpers for ready, missing, and duplicate
  graph snapshots.
- Add a transient-duplicate test that requires two subsequent clean samples.
- Add a clean-streak reset test.
- Add a persistent-duplicate timeout test.
- Add an initially clean two-sample test.
- Preserve coverage for overall readiness timeout and per-command query
  timeout.

Run the focused tests and confirm the new temporal cases fail against the
current immediate-duplicate/single-clean implementation.

## Task 2: Implement the Bounded Settle State Machine

Modify `src/my_course_pkg/my_course_pkg/experiment_session.py`.

- Add validated `SessionConfig` defaults for a 30-second duplicate grace and
  two required clean samples.
- Pass both values into the default `RosGraphProbe`.
- Track the first duplicate deadline and consecutive ready count inside
  `wait_until_ready`.
- Retry transient duplicates, reset the clean streak on every non-ready sample,
  fail when a duplicate remains at its grace deadline, and never extend the
  existing overall deadline.
- Log duplicate-settle start/progress and clean-sample progress only to the
  existing graph log.

Run the focused tests until green.

## Task 3: Regression and Build Verification

- Run all `test_experiment_session.py` tests.
- Run the existing `my_course_pkg` related regression suite used for the
  persistent-session work.
- Compile the changed Python files.
- Build `my_course_pkg` with `colcon build --symlink-install` in the mounted
  Docker workspace.
- Confirm the installed module contains the new defaults and behavior.

## Task 4: Synchronize Documentation

- Copy the approved spec, plan, source, and tests to the source workspace when
  the two repository copies differ.
- Update `docs/agent_handoff.md` in both workspaces with implementation and
  verification status plus the next live-validation command.
- Do not commit unrelated dirty files.

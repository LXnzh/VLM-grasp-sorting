# Vertical Descent XY Tolerance Implementation Plan

**Goal:** Keep vertical pregrasp calibration at `1.5 mm` world-XY error while
allowing the gated descent and final close-readiness verification to accept up
to `2.0 mm`.

**Design:**
`docs/superpowers/specs/2026-07-15-vertical-descent-xy-tolerance-design.md`

## Task 1: Add A Validated Descent Configuration

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/config.py`
- Modify: `src/my_course_pkg/test/test_trajectory_planner.py`

1. Extend the existing vertical-gate default test to require
   `GRASP_VERTICAL_DESCENT_XY_TOLERANCE_M=0.0020`.
2. Extend invalid-value coverage for zero, negative, non-finite, and malformed
   descent tolerance values.
3. Run the focused configuration tests and confirm they fail for the missing
   reader.
4. Add `read_vertical_descent_xy_tolerance_m()` with the same positive finite
   validation used by the existing vertical tolerances.
5. Export `VERTICAL_DESCENT_XY_TOLERANCE_M` without changing the existing
   `VERTICAL_APPROACH_XY_TOLERANCE_M=0.0015` default.
6. Re-run the focused configuration tests.

## Task 2: Route The Correct Tolerance Through Vertical Gates

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/executor.py`
- Modify: `src/my_course_pkg/test/test_grasp_executor_gripper_failures.py`

1. Add failing tests that distinguish strict pregrasp tolerance from descent
   tolerance. Cover pre-command settling, post-command waypoint convergence,
   final verification, errors above `2.0 mm`, and unchanged Z rejection.
2. Add failing assertions that success logs and
   `VerticalApproachConvergenceError` contain the tolerance used by the stage.
3. Import the new descent constant into the executor.
4. Make the vertical success logger, hold-and-raise helper, and convergence
   error receive their XY tolerance explicitly.
5. Pass `VERTICAL_APPROACH_XY_TOLERANCE_M` from pregrasp calibration paths.
6. Pass `VERTICAL_DESCENT_XY_TOLERANCE_M` from lower-waypoint prechecks,
   post-command waypoint gates, and final vertical verification.
7. Preserve `VERTICAL_APPROACH_Z_TOLERANCE_M`, all timeouts, one-command
   waypoint behavior, and hold-before-raise behavior.
8. Re-run the focused executor tests.

## Task 3: Run Regression And Build Verification

**Files:** no additional runtime scope.

1. Run focused trajectory-planner and executor tests with external pytest
   plugin autoload disabled.
2. Run the related grasp selector, pick/place planner, trajectory planner,
   executor, and evaluation regression suite.
3. Compile changed Python modules and tests.
4. Run fatal flake8 checks on changed Python files.
5. Confirm the host and Docker-mounted runtime/test files are synchronized.
6. Run a Docker `colcon build --symlink-install --packages-select
   my_course_pkg` and verify the installed module reports defaults of
   `0.0015`, `0.0020`, and `0.0030` for pregrasp XY, descent XY, and Z.

## Task 4: Record Handoff And Prepare Live Validation

**Files:**

- Modify: `docs/agent_handoff.md`

1. Record the exact changed behavior and verification results.
2. State that no simulator or robot motion ran automatically.
3. Provide a bounded user-run validation: restart the experiment launch with
   `GRASP_DEBUG_STOP_AFTER_CLOSE=1`, run fresh `gelatin_box` and `sponge`
   trials, require descent/final logs at `0.0020`, and stop after close before
   any lift-return trial.

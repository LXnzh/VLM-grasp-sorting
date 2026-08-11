# Vertical Final High-Z Acceptance Implementation Plan

**Goal:** Permit a settled vertical final grasp to close with a positive Z
residual of at most `5.0 mm` only when XY is within `2.0 mm` and target/actual Z
are inside valid live object bounds, while preserving the symmetric `3.0 mm` Z
gate everywhere else.

**Design:**
`docs/superpowers/specs/2026-07-15-vertical-final-high-z-acceptance-design.md`

## Task 1: Add And Validate The Final High-Side Configuration

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/config.py`
- Modify: `src/my_course_pkg/test/test_trajectory_planner.py`

1. Extend the vertical-tolerance default test to require
   `GRASP_VERTICAL_FINAL_HIGH_Z_TOLERANCE_M=0.0050` without changing the
   existing `0.0015/0.0020/0.0030` defaults.
2. Add failing tests for malformed, non-finite, non-positive, and
   below-ordinary-Z values.
3. Run the focused configuration tests and confirm the new assertions fail.
4. Add a positive-finite reader and enforce that the final high-side tolerance
   is no smaller than `GRASP_VERTICAL_APPROACH_Z_TOLERANCE_M`.
5. Export the validated runtime constant and re-run the focused tests.

## Task 2: Model Live Bounds And The Asymmetric Z Decision

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/executor.py`
- Modify: `src/my_course_pkg/test/test_grasp_executor_gripper_failures.py`

1. Add failing unit tests for a pure final-Z decision covering ordinary
   `+/-3.0 mm`, controlled `+4.6 mm`, rejected `-4.6 mm`, greater than
   `+5.0 mm`, missing/invalid bounds, target outside bounds, and actual outside
   bounds.
2. Add a small immutable result structure that carries pass/fail, acceptance
   mode, signed/absolute residual, asymmetric limits, bounds, and rejection
   reason.
3. Extract bottom/center from vertical plan debug data, derive top from the same
   live geometry, and validate finite strict ordering. Preserve an explicit
   disabled reason instead of silently manufacturing bounds.
4. Implement one final-Z predicate. Ordinary `abs(error) <= 3.0 mm` succeeds
   without bounds; controlled success requires `3.0 < signed_error <= 5.0 mm`
   and both target/actual Z inside valid bounds.
5. Re-run the pure decision tests before routing the policy into motion code.

## Task 3: Apply The Policy Only At Final Descent And Close Readiness

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/executor.py`
- Modify: `src/my_course_pkg/test/test_grasp_executor_gripper_failures.py`

1. Add failing executor tests proving a non-final `+4.6 mm` waypoint still
   holds and raises, while the final waypoint waits through the ordinary settle
   window and then accepts the same residual with valid bounds.
2. Add failure coverage for final negative residual, residual above `5.0 mm`,
   invalid/missing bounds, bounds violation, and XY above `2.0 mm`.
3. Extract one final-bounds context from the vertical plan and pass it through
   `move_vertical_approach()` to only its last waypoint and through final
   close-readiness verification.
4. Keep ordinary convergence checks inside the existing settle loop. Evaluate
   controlled-high fallback only at the final waypoint timeout; return without
   sending another lower command when it qualifies.
5. Use the same asymmetric predicate in `verify_vertical_final_grasp()` so a
   settled qualifying waypoint is not rejected immediately before close.
6. Preserve the original symmetric rule for pregrasp, every intermediate
   waypoint, and ordinary final convergence without valid bounds.

## Task 4: Make Diagnostics Match The Safety Decision

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/executor.py`
- Modify: `src/my_course_pkg/test/test_grasp_executor_gripper_failures.py`

1. Add failing assertions for signed residual, negative/positive tolerances,
   bounds, acceptance mode, and precise fallback rejection reason.
2. Extend vertical success logging and `VerticalApproachConvergenceError` to
   receive applicable Z policy details explicitly rather than infer them from
   a stage name.
3. Log controlled-high success distinctly and state that no additional lower
   command was sent.
4. Preserve hold-before-raise and ensure rejected cases never reach
   `close_gripper_at_grasp`.
5. Add or update an execution-sequence test proving close occurs only after the
   final waypoint and close-readiness gates both pass.

## Task 5: Run Regression, Synchronization, And Build Verification

**Files:** no additional runtime scope.

1. Run focused configuration and executor tests with external pytest plugin
   autoload disabled.
2. Run the related selector, pick/place planner, trajectory planner, executor,
   and grasp evaluation regression suites.
3. Compile all changed Python modules and tests.
4. Run fatal flake8 checks on changed Python files.
5. Synchronize only the changed runtime/test files to the Docker-mounted
   `grasp_stable` workspace and confirm host/container hashes match.
6. Run Docker `colcon build --symlink-install --packages-select my_course_pkg`.
7. Confirm the installed module reports pregrasp XY `0.0015`, descent XY
   `0.0020`, ordinary Z `0.0030`, and final high-side Z `0.0050`.

## Task 6: Record Handoff And Prepare Bounded Live Validation

**Files:**

- Modify: `docs/agent_handoff.md`

1. Record the exact implementation, test counts, build result, synchronization
   status, and that no simulator or robot motion ran automatically.
2. Keep implementation changes uncommitted until live validation, consistent
   with the preceding descent-tolerance workflow.
3. Instruct the user to restart the experiment launch with
   `GRASP_DEBUG_STOP_AFTER_CLOSE=1` and run `gelatin_box` first.
4. Require the log to show valid bounds, controlled positive Z acceptance no
   greater than `5.0 mm`, `close_gripper_at_grasp`, and debug stop after close.
5. Repeat close-only validation for `sponge` only after gelatin succeeds; do not
   proceed directly to lift-return.

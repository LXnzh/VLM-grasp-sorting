# Vertical Final Center-Bounded Acceptance Implementation Plan

**Goal:** Replace the experimental fixed `5.0 mm` final high-Z cap with a
live-center boundary while preserving the ordinary symmetric `3.0 mm` path and
all non-final/non-vertical behavior.

**Design:**
`docs/superpowers/specs/2026-07-15-vertical-final-center-bounded-acceptance-design.md`

## Task 1: Retire The Fixed-Cap Configuration

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/config.py`
- Modify: `src/my_course_pkg/test/test_trajectory_planner.py`

1. Update configuration tests so the supported defaults remain pregrasp XY
   `0.0015`, descent XY `0.0020`, and ordinary Z `0.0030` without a final
   fixed-cap constant.
2. Remove `read_vertical_final_high_z_tolerance_m()` and
   `VERTICAL_FINAL_HIGH_Z_TOLERANCE_M`.
3. Remove fixed-cap validation tests and confirm no runtime/operator reference
   still treats the environment variable as authoritative.

## Task 2: Make The Final-Z Decision Center-Bounded

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/executor.py`
- Modify: `src/my_course_pkg/test/test_grasp_executor_gripper_failures.py`

1. Change the shared final-Z helper to evaluate ordinary
   `abs(error) <= 0.0030` first and without requiring bounds.
2. For a positive residual beyond ordinary tolerance, require valid live
   bounds, target inside bottom/top, and actual inside bottom/center.
3. Make center inclusive, remove the fixed-cap comparison, and report
   `center_z - target_z` as the effective positive limit when meaningful.
4. Preserve rejection for negative residuals beyond `3.0 mm`, missing/invalid
   bounds, target outside bounds, actual below bottom, and actual above center.
5. Rename the controlled mode and diagnostic text from `controlled_high` to
   `controlled_center`.

## Task 3: Lock In Non-Regression And Execution Semantics

**Files:**

- Modify: `src/my_course_pkg/test/test_grasp_executor_gripper_failures.py`

1. Prove ordinary positive and negative successes remain bounds-independent
   and keep their existing command/close sequence.
2. Cover the current `gelatin_box` values: target `0.8980 m`, actual
   `0.9032 m`, center about `0.9053 m`; require controlled success.
3. Prove actual exactly at center passes and actual above center fails even
   when below top or within the retired `5.0 mm` cap.
4. Preserve tests for below-bottom actual, out-of-bounds target,
   missing/non-finite/reversed/zero-height bounds, negative residual beyond
   ordinary tolerance, and non-vertical profiles.
5. Prove intermediate waypoints remain symmetric at `3.0 mm`, final fallback
   is evaluated only after settling, and no extra lower command is sent.
6. Prove final waypoint and close-readiness use the same helper and close is
   issued only after both pass.

## Task 4: Verify Diagnostics And Operator Documentation

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/executor.py`
- Modify if needed: `src/my_course_pkg/my_course_pkg/howto.md`
- Modify: `src/my_course_pkg/test/test_grasp_executor_gripper_failures.py`

1. Report decision mode, signed/absolute Z error, ordinary tolerance, bounds,
   effective center-derived positive limit, and a precise rejection reason.
2. Log controlled-center success explicitly and state that no additional lower
   command was sent.
3. Remove current operator references to the retired fixed-cap setting while
   retaining historical specs for rationale.

## Task 5: Run Regression, Build, And Runtime Synchronization

1. Run focused configuration/executor tests with external pytest plugin
   autoload disabled.
2. Run related selector, pick/place planner, trajectory planner, executor, and
   grasp-evaluation regression suites.
3. Compile changed Python files and run fatal flake8 checks.
4. Synchronize only the changed runtime/test files to the Docker-mounted
   `grasp_stable` workspace and confirm hashes match.
5. Run Docker `colcon build --symlink-install --packages-select my_course_pkg`.
6. Confirm installed defaults are pregrasp XY `0.0015`, descent XY `0.0020`,
   and ordinary Z `0.0030`, with no fixed final high-Z constant.

## Task 6: Record Handoff And Prepare Live Validation

**Files:**

- Modify: `docs/agent_handoff.md`

1. Record implementation details, exact test counts, build/synchronization
   status, and that no simulator or robot motion ran automatically.
2. Keep the existing implementation files uncommitted pending live validation,
   consistent with the current experiment workflow.
3. Ask the user to restart the experiment launch and run a fresh close-only
   `gelatin_box` trial with `GRASP_DEBUG_STOP_AFTER_CLOSE=1`.
4. Require valid bounds, `controlled_center`, successful close, and the debug
   stop immediately after close before any lift-return trial.

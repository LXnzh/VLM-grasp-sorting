# Pear Final-Depth Offset Implementation Plan

**Goal:** Raise only pear final/pregrasp targets by 30 mm using a dedicated
world-Z offset while preserving all candidate selection, global convergence,
and non-pear behavior.

**Design:**
`docs/superpowers/specs/2026-07-15-pear-final-depth-offset-design.md`

## Task 1: Freeze Offset Isolation In Tests

**Files:**

- Modify: `src/my_course_pkg/test/test_grasp_selector.py`

1. Assert pear and normalized spelling variants resolve to `+0.010 m`.
2. Parameterize every configured non-pear object and assert it still resolves
   to its existing side, vertical, top-down, or shared offset.
3. Preserve the existing profile-offset tests unchanged.

## Task 2: Add The Pear-Only Offset

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/config.py`
- Modify: `src/my_course_pkg/my_course_pkg/grasp/grasp_selector.py`

1. Add `PEAR_GRASP_Z_OFFSET` from `GRASP_PEAR_Z_OFFSET`, default `0.010`.
2. Normalize the object name in `get_grasp_z_offset` and return the pear value
   only for exact normalized `pear`.
3. Leave every shared offset and profile mapping unchanged.

## Task 3: Verify Prepared Pose Translation

**Files:**

- Modify: `src/my_course_pkg/test/test_grasp_selector.py`

1. Feed a fixed pear object-frame candidate through
   `select_grasp_pose_candidates_6d`.
2. Assert final world Z receives `+0.010 m` and is 30 mm above the legacy
   `-0.020 m` target.
3. Assert world X/Y, rotation, candidate count, score path, and object-frame
   candidate remain unchanged.
4. Assert representative apple preparation still receives `-0.020 m`.

## Task 4: Run Selection And Non-Regression Verification

1. Run the complete selector and pick-place planner suites.
2. Run candidate-ranker, grasp-eval, plan-only, trajectory, and executor suites.
3. Confirm the real 7,466-pose pear test still returns only index 5675 at yaw
   0/180 with unchanged direction and opening metrics.
4. Compile changed Python files and run fatal flake8.
5. Review the diff and confirm no planner, executor, trajectory, perception,
   gripper, tuna, pudding, or shared-offset change was introduced.

## Task 5: Synchronize, Build, And Record Handoff

1. Verify container target files match the host pre-change content, then sync
   only config, selector, and selector tests.
2. Confirm host/container SHA-256 equality.
3. Run Docker `colcon build --symlink-install --packages-select my_course_pkg`.
4. Import the installed package and confirm pear `+0.010 m`, apple `-0.020 m`,
   and unchanged global `18 mm` final tolerance.
5. Update `docs/agent_handoff.md` with implementation, tests, build status, and
   the staged pregrasp/final/close-only live procedure.
6. Keep runtime/test changes uncommitted pending live validation.

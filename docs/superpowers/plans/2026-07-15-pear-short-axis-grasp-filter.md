# Pear Short-Axis Grasp Filter Implementation Plan

**Goal:** Make only `pear` reject long-axis-spanning top-down grasps and retain
the two known short-axis, opening-safe library candidates, without changing any
non-pear selection or execution behavior.

**Design:**
`docs/superpowers/specs/2026-07-15-pear-short-axis-grasp-filter-design.md`

## Task 1: Lock The Pear-Only Configuration Boundary

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/config.py`
- Modify: `src/my_course_pkg/test/test_grasp_selector.py`

1. Add tests for pear-namespaced defaults only:
   `PEAR_MAX_CLOSING_AXIS_ERROR_DEG=5.0` and
   `PEAR_MIN_OPENING_MARGIN_M=0.005`.
2. Add the two constants without modifying shared `ROUND_TOP_*`,
   `SIDE_GRASP_*`, or `VERTICAL_*` values.
3. Verify category/profile mappings and stored geometry remain byte-for-byte
   equivalent for every object.

## Task 2: Add Pure Candidate Geometry Helpers

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/grasp_selector.py`
- Modify: `src/my_course_pkg/test/test_grasp_selector.py`

1. Add a pure helper that projects TCP `X` into the object XY plane and fails
   closed on a near-zero projection.
2. Add a sign-symmetric angle helper comparing that direction with the shorter
   stored horizontal bounding-box axis.
3. Add a conservative projected-AABB width helper:
   `abs(x)*bbox_x + abs(y)*bbox_y`.
4. Test `+short`, `-short`, long-axis, diagonal, and degenerate directions.
5. Test exact pear dimensions: `66.546 mm` on the short axis and `100.455 mm`
   on the long axis.

## Task 3: Route Only Pear Through The New Hard Gates

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/grasp_selector.py`
- Modify: `src/my_course_pkg/test/test_grasp_selector.py`

1. After the existing round-top orientation, center, and height gates, branch
   only when the normalized object name equals `pear`.
2. For pear, reject candidates above the 5-degree short-axis error limit.
3. For pear, reject candidates whose projected width leaves less than 5 mm
   inside the existing 85.16 mm effective opening.
4. Run the existing table-clearance and score ordering on pear survivors.
5. Preserve the existing scalar minimum-width logic exactly for every non-pear
   round-top object.
6. Fail pear closed with a pear-specific reason if no candidate remains; do not
   fall back to generic ranking or synthesize another yaw.

## Task 4: Prove Non-Pear Selection Is Unchanged

**Files:**

- Modify: `src/my_course_pkg/test/test_grasp_selector.py`

1. Capture fixed-input candidate matrices and ordering for representative
   non-pear round-top selection before the implementation.
2. Prove non-pear round-top selection never invokes the pear-only helper.
3. Parameterize routing over all configured non-pear round-top names.
4. Run existing side, vertical, centered, and top-down selector tests unchanged.
5. Confirm no planner, executor, perception, profile mapping, or gripper file is
   changed by the implementation patch.

## Task 5: Add Pear Diagnostics And Real-Library Replay

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/grasp_selector.py`
- Modify: `src/my_course_pkg/test/test_grasp_selector.py`

1. Log pear-only input, direction-survivor, and width-survivor counts.
2. Log each selected pear candidate's short-axis error, projected width, and
   opening margin while leaving the existing non-pear log format unchanged.
3. Replay `grasps/016_pear` with the recorded latest pear scene pose.
4. Assert the previous approximately 69-degree candidate is rejected.
5. Assert every selected candidate is within 5 degrees and has at least 5 mm
   conservative margin.
6. Assert the known raw-index-5675 candidates at 0/180-degree expansion remain
   and appear in existing-score order.

## Task 6: Run Focused Regression And Build

1. Run the complete grasp-selector suite with external pytest plugin autoload
   disabled.
2. Run the pick-place-planner suite to confirm candidate-list integration and
   approach-clearance behavior remain valid.
3. Compile changed Python files and run fatal flake8 checks.
4. Confirm `git diff` contains only the two runtime files, selector tests, and
   handoff update expected by this work, with no unrelated dirty changes staged.
5. Run Docker `colcon build --symlink-install --packages-select my_course_pkg`.
6. Import the installed package and confirm the pear defaults and selector
   helpers are present.

## Task 7: Record Handoff And Prepare Staged Live Validation

**Files:**

- Modify: `docs/agent_handoff.md`

1. Record the exact implementation, test counts, offline real-library result,
   build status, and that no simulator or robot motion ran automatically.
2. Keep runtime/test changes uncommitted until staged live validation, matching
   the repository's current experiment workflow and preserving existing dirty
   work.
3. Ask the user to restart the experiment launch and first run pear only through
   selection/pregrasp/final-without-close inspection.
4. Require fingers along the pear's long sides before close, then perform
   close-only, lift-and-hold, and three fresh-scene full grasp-and-lift trials.
5. If both candidates fail planning, report reachability separately; do not
   relax shared thresholds or modify another object's path.

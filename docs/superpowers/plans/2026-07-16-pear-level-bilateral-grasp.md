# Pear Level Bilateral Grasp Implementation Plan

**Goal:** Synthesize a level, geometry-centered final pose from the two existing
safe pear seeds and reject pear close results that stop before bilateral
contact, without changing any non-pear behavior.

**Design:**
`docs/superpowers/specs/2026-07-16-pear-level-bilateral-grasp-design.md`

## Task 1: Add Pear-Only Bounds

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/config.py`
- Modify: `src/my_course_pkg/test/test_grasp_selector.py`

1. Add 2 mm maximum finger-height difference, 20-degree maximum orientation
   correction, and 0.010 rad close-position calibration tolerance.
2. Change only pear's world-Z offset from +10 mm to +5 mm for the visually
   validated lower bilateral contact.
3. Assert all values are pear-namespaced and shared defaults are unchanged.

## Task 2: Add Pure Level/Center Geometry

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/grasp_selector.py`
- Modify: `src/my_course_pkg/test/test_grasp_selector.py`

1. Project the pear short axis into world XY and construct exact world-down
   right-handed rotations for both seed signs.
2. Preserve seed center-in-TCP Z while zeroing center-in-TCP XY.
3. Compute correction angle and finger-height difference.
4. Fail closed on invalid axes, rotations, centers, or dimensions.
5. Unit-test both signs, centering, levelness, depth preservation, and invalid
   inputs.

## Task 3: Revalidate And Select Synthesized Candidates

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/grasp_selector.py`
- Modify: `src/my_course_pkg/test/test_grasp_selector.py`

1. Keep the existing pear direction/width seeds.
2. Synthesize one level candidate from each seed.
3. Apply the approved correction, levelness, direction, width, center, height,
   world-down, and table gates to final candidates.
4. Retain both 180-degree alternatives and existing score ordering.
5. Add diagnostics and real-library replay assertions.

## Task 4: Carry Candidate Close Geometry Into The Plan

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/pick_place_planner.py`
- Modify: `src/my_course_pkg/test/test_pick_place_planner.py`

1. Recompute final pear width from the selected object-frame candidate.
2. Derive expected and minimum close positions using the approved formula.
3. Store finite pear-only values in plan debug information.
4. Assert non-pear plans do not gain pear execution behavior.

## Task 5: Enforce Pear Close Readiness Before Lift

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/executor.py`
- Modify: `src/my_course_pkg/test/test_grasp_executor_gripper_failures.py`

1. After the existing generic close validation, inspect pear's actual close
   position.
2. Reject missing, non-finite, zero, `0.038 rad`, and any result below the
   candidate-specific minimum before hold or lift.
3. Accept a pear contact at or above the minimum.
4. Prove identical close results retain existing behavior for non-pear plans.

## Task 6: Verify Regression And Installed Runtime

1. Run focused pear selector, planner, trajectory, executor, and non-target
   tests with external pytest plugin autoload disabled.
2. Run the broader related grasp regression and record known unrelated
   failures separately.
3. Compile changed Python files and run fatal flake8.
4. Confirm the diff contains no shared-default or non-pear path change.
5. Synchronize only changed runtime/test files to the active container if
   needed, run the symlink package build, and verify installed constants and
   real-library replay.

## Task 7: Record Safe Live Stages

**Files:**

- Modify: `docs/agent_handoff.md`

1. Record implementation, exact test/build results, and that no motion ran.
2. Give final-pose, close-only, 30 mm test-lift, and full-lift commands.
3. Require visual levelness and bilateral contact before each next stage.
4. Keep implementation changes uncommitted pending staged live validation.

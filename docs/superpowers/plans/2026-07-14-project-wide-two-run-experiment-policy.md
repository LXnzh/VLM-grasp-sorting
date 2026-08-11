# Project-Wide Two-Run Experiment Policy Implementation Plan

> Implement the approved design in
> `docs/superpowers/specs/2026-07-14-project-wide-two-run-experiment-policy-design.md`.
> This plan changes qualification counts only. It does not authorize simulator
> startup, `grasp_demo`, arm motion, Cartesian servo, or gripper commands.

## Goal

Make two consecutive independent successes the project-wide qualification
minimum while preserving all existing safety gates, failure-reset semantics,
fresh-input requirements, and optional longer diagnostic runs.

## Constraints

- Preserve every unrelated dirty or untracked workspace change.
- Use focused semantic patches; never replace every `3` or every occurrence of
  “three” mechanically.
- Keep historical experiment records factual.
- Do not change collision margins, geometry, scene layout, simulator physics,
  controller settings, hold duration, or motion paths.
- Do not run Phase B. The retained real-motion health gate is `>=18 Hz`, while
  the latest accepted live state cadence is about `3.43 Hz`.

## Task 1: Lock The Two-Success Semantics In Tests

**Files:**

- Modify: `src/my_course_pkg/test/test_grasp_eval.py`

1. Change the default-trial test to expect `2`.
2. Keep the explicit longer-trial-count test.
3. Replace the three-success streak fixtures with cases proving:
   - one success does not qualify;
   - two consecutive successes qualify;
   - a failure resets the current streak;
   - a later two-success streak can qualify.
4. Change summary expectations from `2/3`, `3/3` to the corresponding
   two-success results.
5. Run only `test_grasp_eval.py` and confirm the tests fail against the current
   runtime constant of `3` for the intended reason.

## Task 2: Implement The Runtime Policy

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp_eval.py`

1. Change `REQUIRED_CONSECUTIVE_SUCCESSES` from `3` to `2`.
2. Keep `compute_consecutive_streaks()` pure and preserve its current/max
   streak behavior.
3. Keep `--trials N` available for explicit longer diagnostic batches.
4. Keep the default `--trials` value tied to the policy constant.
5. Rerun `test_grasp_eval.py`; require a pass.

## Task 3: Update Active Normative Documentation

**Files:**

- Modify: `docs/superpowers/specs/2026-07-13-apple-stable-grasp-roadmap-design.md`
- Modify: `docs/superpowers/specs/2026-07-13-scene-clearance-bounds-and-three-run-validation-design.md`
- Modify: `docs/superpowers/plans/2026-07-13-scene-clearance-bounds-and-three-run-validation.md`
- Modify: `docs/superpowers/specs/2026-07-14-exact-obb-approach-corridor-design.md`
- Modify: `docs/agent_handoff.md`

1. Change active Apple and project qualification requirements from three
   consecutive successes to two.
2. Reduce active trial tables to two formal rows where the rows represent the
   required qualification count.
3. Mark the older three-run design and plan as superseded by the approved
   project-wide two-run policy. Preserve their historical implementation
   descriptions rather than rewriting every old checklist item.
4. Update the exact-bounds design's current status and forward-looking stage
   requirements to `2/2`.
5. Keep historical A1 results that actually contain three captures unchanged.
6. Keep dimensions, three-dimensional error language, five-second holds, five
   object categories, trial IDs, and unrelated numeric values unchanged.
7. Update the handoff Current Snapshot and add a concise implementation result
   only after verification completes.

## Task 4: Offline Verification

Use system ROS Python, not the MuJoCo virtual environment:

```bash
cd /home/ws
source /opt/ros/humble/setup.bash
source install/setup.bash

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  src/my_course_pkg/test/test_grasp_eval.py -q

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  src/my_course_pkg/test/test_grasp_selector.py \
  src/my_course_pkg/test/test_pick_place_planner.py \
  src/my_course_pkg/test/test_grasp_plan_only.py \
  src/my_course_pkg/test/test_grasp_executor_gripper_failures.py \
  src/my_course_pkg/test/test_trajectory_planner.py \
  src/my_course_pkg/test/test_grasp_eval.py -q

colcon build --packages-select my_course_pkg --symlink-install
```

No simulator, perception pipeline, MoveIt goal, arm motion, or gripper command
is part of these checks.

## Task 5: Residual Policy Audit

1. Scan active source, tests, specs, plans, and the handoff for qualification
   language that still requires three successes.
2. Classify every remaining match as one of:
   - historical fact that must remain unchanged;
   - superseded design/plan text with a prominent policy reference;
   - unrelated numeric or three-dimensional language;
   - an active normative defect that must be fixed.
3. Verify `git diff --check` on every changed file.
4. Confirm only the intended policy files changed during this implementation.

## Task 6: Handoff And Next Gate

1. Record the focused and related test counts and build result.
2. Record Apple A3 as complete under the `2/2` policy.
3. Record that Phase B is still blocked by the `>=18 Hz` real-motion health
   gate.
4. Make cadence recovery the next separately designed action. Do not emit or
   run Phase B motion commands until that design is approved, implemented, and
   verified at the required rate.

## Definition Of Done

- Runtime evaluator default and qualification streak use `2`.
- Focused tests prove one-fail/two-pass/reset/recovery behavior.
- Explicit longer diagnostic batches remain supported.
- Active normative documentation uses or explicitly defers to the two-run
  policy.
- Historical results remain factual.
- Related regression tests and `my_course_pkg` build pass.
- No safety threshold, scene, simulator, perception, planning, motion, or
  gripper behavior changes.
- Phase B remains locked pending the `>=18 Hz` health gate.

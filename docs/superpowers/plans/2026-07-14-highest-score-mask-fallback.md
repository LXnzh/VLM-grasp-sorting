# Highest-Score Multi-Mask Fallback Implementation Plan

Date: 2026-07-14
Design: `docs/superpowers/specs/2026-07-14-highest-score-mask-fallback-design.md`

## Objective

Implement a global, fully automatic multi-mask fallback in the active runtime
workspace and synchronize it to the original workspace. A valid high-confidence
VLM verifier choice still wins. When the enabled verifier produces no usable
choice, select by finite `grounding_score`, then finite `score`, then candidate
index. Explicit `FOUNDATIONPOSE_AUTO_MASK_VERIFY=0` remains fail-closed.

## Task 1: Add Failing Focused Tests

Files:

- `src/my_course_pkg/test/test_foundationpose_mask_matching.py`

Add tests for:

1. low-confidence verifier output selecting the highest finite
   `grounding_score` candidate;
2. verifier exception/error using the same fallback;
3. absent/non-finite grounding scores falling back to finite `score`;
4. all scores absent/non-finite choosing the lower candidate index;
5. score ties choosing the lower candidate index;
6. fallback metadata recording
   `selected_by: highest_detection_score_fallback`;
7. `FOUNDATIONPOSE_AUTO_MASK_VERIFY=0` preserving the current multi-mask
   RuntimeError and producing no selected-mask metadata;
8. tomato-can ambiguity using the same fallback and recording its source;
9. existing high-confidence verifier, single-match, no-match, and manual
   override behavior remaining unchanged.

Run the focused file before implementation and confirm that only the new
fallback expectations fail.

```bash
cd /home/ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q \
  src/my_course_pkg/test/test_foundationpose_mask_matching.py
```

## Task 2: Implement the Pure Ranking Helper

Files:

- `src/my_course_pkg/my_course_pkg/perception/foundationpose.py`

Add a selection-source constant and a small pure helper that:

1. reads candidate metadata only;
2. ranks globally by finite numeric `grounding_score` when any candidate has
   one;
3. otherwise ranks by finite numeric `score` when any candidate has one;
4. otherwise chooses the lower candidate index;
5. puts missing/non-finite values below finite values and resolves score ties
   by lower index.

Do not change the verifier request, retry behavior, target aliases, or mask
generation.

## Task 3: Connect the Fallback to Multi-Mask Loading

Files:

- `src/my_course_pkg/my_course_pkg/perception/foundationpose.py`

In the existing `len(candidates) > 1` branch:

1. preserve valid manual override handling;
2. preserve fail-closed behavior when
   `FOUNDATIONPOSE_AUTO_MASK_VERIFY=0`;
3. use a valid high-confidence verifier candidate when present;
4. otherwise call the pure ranking helper;
5. write selected-mask metadata with
   `highest_detection_score_fallback` and return that mask.

Preserve `mask_verification_result.json` from the attempted verifier. Do not
add a VLM call or a score-margin check.

## Task 4: Verify and Synchronize Both Workspaces

Apply the same semantic code and test edits to:

- `E:\IFL\ros2-docker-workspace-vscode-plmrs-grasp-stable` (active `/home/ws`)
- `E:\IFL\ros2-docker-workspace-vscode-plmrs` (original)

Run in the container:

```bash
cd /home/ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q \
  src/my_course_pkg/test/test_foundationpose_mask_matching.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q \
  src/my_course_pkg/test
colcon build --packages-select my_course_pkg --symlink-install
```

Run `git diff --check` on the changed source and test files in both workspaces,
and compare them semantically while allowing existing line-ending differences.
Do not commit unrelated dirty-worktree changes.

## Task 5: Resume Live Qualification

No API key belongs in logs or chat. With the low-rate simulator running:

1. unset `FOUNDATIONPOSE_MASK_INDEX` and `VLM_CANDIDATE_OVERRIDE`;
2. perform independent `/reset_sim` + fresh pipeline A1 Trial 1;
3. verify the selected mask is the actual Apple, selection metadata is
   automatic, pose is current, and geometry-center error is below `10 mm`;
4. repeat for independent A1 Trial 2;
5. only after A1 reaches `2/2`, run two A3 `grasp_plan_only` trials.

No `grasp_demo`, trajectory execution, arm motion, or gripper command is
authorized by this plan.

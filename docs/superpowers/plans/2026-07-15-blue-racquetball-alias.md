# Blue Racquetball Fixed Alias Implementation Plan

**Goal:** Accept `blue racquetball` as a deterministic alias for canonical YCB
object `racquetball`, ground the image with `blue ball.`, retain the racquetball
CAD/grasp identity, and fail closed when the alias mask is uncertain.

## Step 1: Alias selection tests

Add `src/my_course_pkg/test/test_target_aliases.py` first. Cover full-word,
case-insensitive and flexible-whitespace matching, rejection of partial words,
the exact canonical identity/prompt/class metadata, and the absence of a match
for ordinary instructions.

Add focused `LlmSam2Node.run()` tests with network calls replaced. Require:

- `pick up the blue racquetball` bypasses VLM selection;
- `selected_object_name` is `racquetball`;
- the SAM request prompt is exactly `blue ball.`;
- alias audit fields are written to `selected_object.json`;
- `VLM_CANDIDATE_OVERRIDE` retains precedence and existing behavior.

Run the new tests and require RED before implementation:

```bash
cd /home/ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  src/my_course_pkg/test/test_target_aliases.py -q
```

## Step 2: Alias resolver and LLM/SAM2 integration

Add `src/my_course_pkg/my_course_pkg/perception/target_aliases.py` as a small,
side-effect-free alias registry/resolver. Modify
`src/my_course_pkg/my_course_pkg/perception/llm_sam2.py` to separate canonical
identity from grounding prompt and accepted SAM class names.

Keep the explicit environment override first, the fixed alias second, and the
existing VLM path third. Ordinary instructions must retain the current JSON and
prompt behavior. Run the Step 1 tests until GREEN.

## Step 3: Dynamic SAM class matching and strict alias failure tests

Extend `src/my_course_pkg/test/test_foundationpose_mask_matching.py` first.
Require that alias metadata lets a `blue ball` annotation match canonical
target `racquetball`, while `_mesh_dir()` still resolves the racquetball mesh.
Add alias-path tests proving multiple-candidate verifier uncertainty, invalid
selection, and verifier failure do not use the highest-score fallback. Preserve
the existing fallback tests for ordinary targets.

Modify `src/my_course_pkg/my_course_pkg/perception/foundationpose.py` so
`_sam2_class_matches_target()` explicitly accepts and normalizes the dynamic
`accepted_sam2_class_names` loaded from `selected_object.json`. This function
extension is mandatory: changing only the prompt would cause every returned
`blue ball` annotation to be filtered out before mask selection.

Carry the alias strictness and visual description through mask filtering and
verification without using `blue ball` as the mesh name. Invalidate stale pose
output before any mask failure and do not call the external pose service on a
failed alias mask.

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  src/my_course_pkg/test/test_foundationpose_mask_matching.py -q
```

## Step 4: Related regression and build verification

Run the focused perception tests, related grasp tests, compilation, and a
symlink package build:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  src/my_course_pkg/test/test_target_aliases.py \
  src/my_course_pkg/test/test_foundationpose_mask_matching.py \
  src/my_course_pkg/test/test_grasp_selector.py \
  src/my_course_pkg/test/test_pick_place_planner.py -q

/usr/bin/python3 -m py_compile \
  src/my_course_pkg/my_course_pkg/perception/target_aliases.py \
  src/my_course_pkg/my_course_pkg/perception/llm_sam2.py \
  src/my_course_pkg/my_course_pkg/perception/foundationpose.py

colcon build --packages-select my_course_pkg --symlink-install
```

Synchronize the runtime and test files to the Docker-mounted active workspace
if the implementation is performed in the host workspace.

## Step 5: Saved-frame non-motion validation and handoff

Use the saved RGB frame from the failed racquetball run. Run only the alias
selection, SAM2 mask generation, and FoundationPose inputs; do not run
`grasp_demo` and do not start arm motion. Require the blue ball to appear in the
candidate overlay, the yellow tennis ball not to be selected, and the pose
request bundle to contain the racquetball mesh.

Update `docs/agent_handoff.md` with code state, exact test/build results, any
external-service limitation, and the next safe plan-only validation step.

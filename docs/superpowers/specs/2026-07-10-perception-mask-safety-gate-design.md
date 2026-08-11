# Perception Mask Safety Gate Design

## Goal

Prevent an ambiguous or wrong SAM2/GroundingDINO mask from driving
FoundationPose and arm motion. This is a perception-layer safety gate, not a
temporary workaround: when the detector returns multiple plausible target
masks, the safest behavior is to stop and require a human-visible selection.

## Evidence

The 2026-07-10 tomato-can run returned four annotations matching the target
class. The first annotation was an unrelated round fruit with the highest
GroundingDINO score, while the real tomato-can top was a later, lower-scored
annotation. `FoundationPoseEstimationNode._load_mask()` filtered by class name
and then unconditionally used `anns[0]`, so FoundationPose estimated the pose
of the wrong object and the robot moved toward that wrong pose.

This failure cannot be fixed reliably by grasp planning, final approach
tuning, or gripper behavior. Those later stages only receive the object pose
that FoundationPose produced from the selected mask.

## Options Considered

1. Fail closed on ambiguous masks and allow only explicit mask override.
   This is the selected option. It makes wrong-target execution impossible by
   default and keeps the operator in the loop when the perception output is
   visibly ambiguous.
2. Pick the highest-confidence detection automatically.
   Reject this for now. The observed tomato-can failure had the wrong object at
   the highest score, so confidence alone would preserve the bug.
3. Add heuristic reranking using bbox size, image position, or object priors.
   This may be useful later, but it would be hard to trust before the system
   records candidates clearly and fails safely on ambiguity.

## Scope

- Change only the FoundationPose mask selection boundary in phase 1.
- Keep VLM target selection, SAM2 API calls, FoundationPose service calls, grasp
  selection, arm motion, and gripper behavior unchanged unless they are needed
  to surface the new mask-selection error cleanly.
- Do not automatically choose among multiple matching masks without an explicit
  operator override.
- Do not use confidence score as a tie breaker for tomato/tuna cans in this
  phase.

## Design

### Mask Candidate Extraction

`FoundationPoseEstimationNode._load_mask()` will keep the existing class-name
matching and aliases, but it will build structured candidate metadata before
selecting a mask:

- `index`: 1-based candidate index in the filtered target-matching list.
- `source_annotation_index`: 0-based index in the raw SAM2 annotation list.
- `class_name`: SAM2/GroundingDINO class label.
- `score`: annotation confidence when present.
- `bbox`: annotation box when present.
- `area_px`: decoded mask area in pixels.

The metadata will be written under the FoundationPose output directory as
`mask_candidates.json` on every run that reaches mask loading.

### Selection Rules

The default behavior is:

1. Zero matching masks: raise `RuntimeError` with the target name and the
   candidate summary path.
2. One matching mask: select it and continue.
3. Multiple matching masks: raise `RuntimeError` explaining that perception is
   ambiguous and that arm motion must not continue until a mask is explicitly
   selected.

An operator can opt in to a specific candidate by setting
`FOUNDATIONPOSE_MASK_INDEX` to the 1-based candidate index from
`mask_candidates.json`. When set:

- the index must be an integer;
- it must refer to one of the filtered target-matching candidates;
- invalid values raise `RuntimeError`;
- the selected candidate metadata is written to
  `selected_mask_metadata.json`.

This makes manual selection reproducible in logs and avoids hidden automatic
rules.

### Visual Diagnostics

When multiple matching masks exist, the pipeline already saves
`grounded_sam2_annotated_image_with_mask.jpg`. Phase 1 will additionally save a
compact candidate metadata file that maps candidate numbers to bbox, score, and
area. It will also save `mask_candidates_overlay.jpg` with candidate numbers
drawn near their boxes when bbox data is present. If a candidate lacks bbox
data, the metadata file remains authoritative for that candidate.

### Pipeline Behavior

`pipeline` will naturally stop when `FoundationPoseEstimationNode.run()` raises
the ambiguity error. The error text must be actionable:

- name the target object;
- report the matching candidate count;
- point to `mask_candidates.json`;
- mention `FOUNDATIONPOSE_MASK_INDEX=<n>` as the explicit override mechanism.

To prevent accidental reuse of stale perception, `FoundationPoseEstimationNode`
will remove or invalidate the previous `pose_result.json` before mask selection
and will write a new success result only after FoundationPose succeeds. If mask
selection fails, `grasp_demo` must not find a fresh pose result from that failed
pipeline run.

## Error Handling

The gate fails before the FoundationPose HTTP request is sent. The error message
and handoff/debug docs must remind the operator not to run `grasp_demo` after a
failed or ambiguous pipeline. Removing or invalidating the old pose result is
part of the safety gate, because an ambiguous perception run must not leave a
usable stale pose behind.

## Tests

Extend `test_foundationpose_mask_matching.py`:

- zero target matches raises a clear error;
- one alias match still returns the expected mask;
- multiple matches without override raises ambiguity and writes candidate
  metadata;
- `FOUNDATIONPOSE_MASK_INDEX` selects the requested filtered candidate;
- invalid override values raise and do not silently fall back;
- selected metadata records the chosen candidate;
- ambiguous or invalid selection removes or invalidates the previous
  `pose_result.json`.

Run the focused mask tests with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`. A full
`my_course_pkg` build is useful after implementation, but the phase 1 behavior
is primarily covered by unit tests.

## Live Verification

1. Reset the simulator.
2. Run `pipeline` for the tomato can.
3. If SAM2 returns more than one tomato/tomato-soup match, confirm the pipeline
   stops before FoundationPose and writes candidate metadata.
4. Inspect the SAM2 annotated image and candidate metadata.
5. Rerun `pipeline` with `FOUNDATIONPOSE_MASK_INDEX=<real-can-index>`.
6. Confirm FoundationPose returns the known tomato-can camera pose near
   `[-0.4307, -0.1005, 0.6929]` before running any grasp command.

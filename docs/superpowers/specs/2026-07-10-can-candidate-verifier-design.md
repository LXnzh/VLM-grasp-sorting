# Can Candidate Verifier Design

## Goal

Make `pick up the tomato soup can` select the actual can automatically when
SAM2/GroundingDINO returns multiple target-labeled masks. The manual
`FOUNDATIONPOSE_MASK_INDEX` override remains a debug escape hatch, but normal
operation should not require it.

This phase only selects the correct perception mask. It does not change
FoundationPose, camera TF handling, grasp selection, final approach, gripper
control, or lift verification.

## Current Blocker

The current safety gate correctly refuses to continue when multiple masks match
`tomato soup can`. In the latest live run, candidate `#1` was the tomato soup
can and candidate `#2` was an apple, but both were labeled `tomato soup` by
SAM2/GroundingDINO. The system therefore stopped before FoundationPose.

That is safe, but not yet the desired user experience. The desired behavior is:

```text
pick up the tomato soup can
-> multiple target-labeled masks
-> automatically identify the can among the candidates
-> run FoundationPose only on that mask
```

If the automatic verifier is not confident, the existing fail-closed behavior
must remain.

## Options Considered

1. VLM candidate verifier on numbered crops.
   This is the selected option. It asks the same vision-capable model used by
   the pipeline to choose among numbered candidate crops. It directly checks
   appearance rather than trusting SAM2 labels or scores.
2. Heuristics using score, area, bbox location, or object size.
   Reject for now. Recent failures already showed that confidence can prefer
   the wrong object, and object placement varies across the scene.
3. Keep only manual `FOUNDATIONPOSE_MASK_INDEX`.
   Safe but too clumsy for normal operation. It should remain available for
   debugging and recovery, not be the default interaction.

## Design

### Trigger

The verifier runs inside `FoundationPoseEstimationNode._load_mask()` after
candidate metadata has been written:

1. If `FOUNDATIONPOSE_MASK_INDEX` is set, use the explicit override first.
2. If exactly one target-matching candidate exists, keep the current automatic
   single-candidate path.
3. If multiple candidates exist and auto verification is enabled, run the VLM
   candidate verifier.
4. If the verifier returns a valid high-confidence index, select that mask and
   continue.
5. Otherwise raise the existing ambiguity error and do not run FoundationPose.

Auto verification should be enabled by default because the pipeline already
requires a vision-capable VLM key for target selection. Add
`FOUNDATIONPOSE_AUTO_MASK_VERIFY=0` as a disable switch for debugging.

### Candidate Card Image

Build a compact candidate-card image from the saved RGB frame:

- one panel per candidate;
- crop each panel around its bbox with a small margin;
- draw a large visible candidate number on each panel;
- include only candidates that match the selected target class;
- save the card to `foundationpose/mask_verification_candidates.jpg`.

The card is the image sent to the VLM. The existing
`mask_candidates_overlay.jpg` remains useful for human review in scene context.

### VLM Prompt

Reuse the existing OpenAI-compatible VLM client configuration from
`llm_sam2.py` so the verifier uses the same API key, base URL, model, retry
behavior, image support checks, and JSON parsing style as target selection.

The prompt should be strict:

- target object: the selected YCB object name, for this phase especially
  `tomato soup can`;
- input image: numbered candidate crops;
- instruction: choose the candidate that is the target object;
- allowed output: JSON only;
- if none or uncertain, return `{"selected_index": null, "confidence": "low"}`;
- if certain, return
  `{"selected_index": <candidate number>, "confidence": "high"}`.

The verifier should not accept free text, object names, or out-of-range
indices.

### Acceptance Rules

The verifier may automatically select a mask only when all of these are true:

- response parses as JSON;
- `selected_index` is an integer matching one filtered candidate;
- `confidence` is exactly `"high"`;
- no manual override conflicts with the selected index.

Any API error, parse error, missing image support, low confidence, null index,
or invalid index fails closed with the same actionable ambiguity message. The
error message should mention the saved candidate card and the manual
`FOUNDATIONPOSE_MASK_INDEX=<n>` fallback.

### Metadata

Write `mask_verification_result.json` on every verifier attempt:

- target object;
- candidate count;
- candidate-card path;
- raw VLM response text;
- parsed selected index when available;
- confidence;
- final decision: `selected`, `uncertain`, or `error`;
- selected candidate metadata when selected.

When the verifier selects a mask, write `selected_mask_metadata.json` with
`selected_by: "vlm_candidate_verifier"`.

## Tests

Extend the focused FoundationPose mask tests with fake verifier behavior:

- multiple candidates with high-confidence verifier index selects that mask;
- verifier low confidence fails closed;
- verifier invalid index fails closed;
- manual `FOUNDATIONPOSE_MASK_INDEX` bypasses the verifier;
- selected metadata records `vlm_candidate_verifier`;
- verifier result metadata is written for selected and rejected cases.

Add any helper tests for candidate-card generation that can run without a live
VLM service.

## Live Verification

1. Reset and run `pipeline` for `pick up the tomato soup can` without
   `FOUNDATIONPOSE_MASK_INDEX`.
2. Confirm `mask_candidates.json` contains the two recent candidates.
3. Confirm `mask_verification_candidates.jpg` shows numbered crops.
4. Confirm `mask_verification_result.json` selected candidate `#1` with high
   confidence.
5. Confirm `selected_mask_metadata.json` says
   `selected_by: "vlm_candidate_verifier"`.
6. Confirm FoundationPose succeeds and `pose_result.json` is for the visible
   tomato soup can before running any grasp motion.

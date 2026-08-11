# Highest-Score Multi-Mask Fallback Design

Date: 2026-07-14

## Context

The Apple A1 pipeline sometimes receives two SAM2 annotations that are both
labeled `apple`: the red Apple and a yellow lemon. The existing VLM candidate
verifier has returned `low`, then `high`, then `low` on equivalent captures.
Its fail-closed behavior is safe, but it makes the student experiment
unnecessarily difficult to repeat. In all inspected captures, the correct
Apple also had the higher GroundingDINO detection score.

The user explicitly chose a simpler student-project rule: the pipeline must
remain automatic, but when the verifier is uncertain it should select the
matching candidate with the highest detection score.

## Goal

Allow the perception pipeline to continue automatically when multiple masks
match the selected target and the VLM verifier does not return a usable
selection.

## Non-Goals

- Do not add a second VLM request.
- Do not add Apple-, lemon-, or color-specific selection rules.
- Do not require a minimum score margin.
- Do not change SAM2, FoundationPose, grasp selection, or motion behavior.
- Do not treat a manual `FOUNDATIONPOSE_MASK_INDEX` run as an automatic A1 run.

## Selection Behavior

1. If no mask matches the selected target, preserve the existing error.
2. If exactly one mask matches, preserve the existing direct selection.
3. If a valid manual override is present, preserve the existing diagnostic
   override behavior.
4. If multiple masks match and `FOUNDATIONPOSE_AUTO_MASK_VERIFY=0`, preserve
   the existing fail-closed behavior. Explicitly disabling automatic mask
   verification must not silently enable score-based selection.
5. Otherwise, run the existing VLM verifier once.
6. If the verifier returns a valid candidate index with `high` confidence, use
   that candidate and record `selected_by: vlm_candidate_verifier`.
7. Otherwise, inspect score fields in this global order. If at least one
   candidate has a finite numeric `grounding_score`, rank by that field and put
   missing/non-finite values below finite values. If no candidate has a finite
   `grounding_score`, repeat the same rule with `score`. If neither field has a
   finite value for any candidate, choose by candidate index. Higher scores win;
   ties choose the lower candidate index.
8. Record the fallback as
   `selected_by: highest_detection_score_fallback` together with the complete
   selected-candidate metadata.

The fallback also applies when the verifier returns malformed data or its API
request fails, because those paths already collapse to no verifier candidate.
The original verifier result remains available in
`mask_verification_result.json` for diagnosis.

This is a global multi-mask policy, not an Apple-only exception. It therefore
also changes tomato-can behavior when the verifier is enabled but uncertain or
fails: tomato is no longer guaranteed to stop at that point and may use the
highest-scoring matching mask. Tomato regression evidence must be recorded
before qualification continues. This design still authorizes no motion.

## Code Boundary

Keep the change inside
`my_course_pkg/perception/foundationpose.py`. Add a small pure helper that
chooses the highest-scoring candidate, then call it from the existing
multi-candidate branch only when `_selected_candidate_from_verifier()` returns
no candidate. Do not refactor unrelated perception code.

## Verification

Focused tests must cover:

- verifier high-confidence selection still wins;
- verifier low-confidence result falls back to the highest grounding score;
- verifier error or invalid result uses the same fallback;
- `FOUNDATIONPOSE_AUTO_MASK_VERIFY=0` preserves fail-closed behavior;
- ranking uses finite `grounding_score`, then finite `score`, then candidate
  index, with deterministic tie handling;
- equal scores choose the first filtered candidate deterministically;
- fallback metadata uses `highest_detection_score_fallback`;
- single-match, no-match, and manual-override behavior remain unchanged.

Add a tomato-can multi-mask regression proving that verifier uncertainty uses
the same global fallback and records its selection source. This test documents
the intentionally relaxed behavior; it does not assert that score alone proves
semantic correctness.

Run the focused FoundationPose mask tests, the related perception/package test
suite, and a package build. After implementation, restart the live qualification
counter and perform two independent Apple A1 reset + fresh-pipeline trials.
Each run must select the actual Apple and produce a current successful pose.
Only after A1 reaches `2/2` may the project run two A3 plan-only trials. This
design authorizes no trajectory execution or gripper command.

## Rollback

If the fallback selects the wrong object during either A1 trial, restore the
previous fail-closed multi-mask behavior and reassess the selection rule. Do
not compensate for a wrong mask in FoundationPose or grasp parameters.

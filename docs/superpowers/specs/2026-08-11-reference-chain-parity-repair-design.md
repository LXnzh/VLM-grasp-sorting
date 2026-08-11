# Reference-Chain Parity Repair Design

Date: 2026-08-11

Status: Approved in conversation

## Goal

Repair `E:\IFL\VLM_grasp_stable` without designing another orchestration
pipeline. The already-run dynamic product chain in
`E:\IFL\PraktikumSoSe26_ML for Robotics` is the source of truth for GUI,
VLM/SAM2, tracking, PBVS, stable-frame FoundationPose, and sorting flow. The
validated runtime in `E:\IFL\grasp_stable_pure` is the source of truth for
grasp selection, planning, guarded execution, lift, release, and return-home
behavior.

Pinned source trees:

- dynamic chain: `98f386d9c1b3f4b6d7401b93e97c6ed263ebcf40`;
- stable grasp runtime: `c3e7a66`.

## Root Cause

The integration copied `_DirectMaskFoundationPose` from the dynamic reference
but omitted its `FoundationPoseEstimationNode._validate_mask_geometry()`
dependency and the corresponding border-mask tests. Integration tests mocked
the real stable-frame-to-FoundationPose boundary, so the incomplete port passed
the non-actuating suite and failed only in the GUI run.

## Implementation Boundary

Do not invent, redesign, or independently reimplement the dynamic chain.
Mechanically compare it with the pinned dynamic reference and synchronize every
runtime dependency required by the selected product path. Preserve module
splits only when they are behaviorally identical and covered by parity tests;
otherwise use the reference implementation directly.

Do not replace stable grasp code with the feature branch's reduced grasp
implementation. Mechanically compare the target grasp runtime with `c3e7a66`.
Allowed semantic differences are limited to:

1. removal of unsupported Tuna/Pudding product routes; and
2. the typed classified-placement suffix after the stable grasp-and-lift
   prefix.

Any other difference must be removed or explicitly justified by a passing
regression test and recorded in `HANDOFF.md`.

## Verification

Add a non-mocked contract test that exercises the concrete stable-frame direct
mask class through the FoundationPose input boundary. Restore the reference
mask-area and image-border tests. Run the stable selector, planner, trajectory,
executor, gripper, tracking, PBVS, sorting, and GUI suites in the Dev Container,
then rebuild `my_course_pkg` and the full workspace as needed.

Final acceptance is one fresh GUI-driven MuJoCo banana trial that reaches
FoundationPose, stable planning, grasp, lift, classified placement, release,
retreat, and verified return home. Do not weaken safety thresholds to obtain a
pass.

## Handoff

Update the root `HANDOFF.md` with the two source commits, the allowed-difference
list, the original failure and missing-test cause, exact verification commands,
test/build results, live-run artifacts, and any remaining blocker. This record
must make rebuilding the unchanged broken tree or reimplementing the reference
chain an explicit do-not-repeat item.

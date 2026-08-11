# VLM Tracking, Stable Grasp, and Sorting Implementation Plan

Date: 2026-08-11

Design:
`docs/superpowers/specs/2026-08-11-vlm-tracking-stable-grasp-sorting-design.md`

Branch: `codex/integrate-stable-tracking-sorting`

## Outcome

Produce a complete ROS 2 workspace in `E:\IFL\VLM_grasp_stable` that runs the
GUI/voice -> VLM/SAM2 -> tracking/PBVS -> stable grasp -> classified placement
flow for the 16 supported objects and passes full MuJoCo acceptance.

## Task 1: Record the Three Source Baselines

- Record the target pre-integration tree at `fb4878d`.
- Record the stable workspace tree at `7bbfd07` and the semantic runtime commits
  named in the design.
- Record the feature workspace tree at `98f386d` and food-scene commit
  `3a203c9`.
- Export lists of target-only dynamic source/test files and stable tracked files
  so the mechanical import is auditable.
- Verify all three source worktrees before copying; never mutate either source
  workspace.

## Task 2: Replace the Package Snapshot with the Stable ROS Workspace

- Remove the target's old root `my_course_pkg`, orphan `base_env.yaml`, and
  generated/cache files from version control.
- Import committed files from stable commit `7bbfd07`, excluding its Git state
  and repository-specific dirty/untracked files.
- Preserve this project's design/plan documents and current GUI source for the
  later port.
- Rewrite copied project-level instructions and README where pure-product scope
  conflicts with the sorting product.
- Commit the mechanical workspace import separately.

## Task 3: Add Runtime Data to Git

- Copy the grasp libraries and numbered YCB runtime assets required by the 16
  supported objects.
- Remove Tuna/Pudding runtime data and any duplicate alias trees not required by
  the canonical numbered-asset loader.
- Update `.gitignore` so these two intentional data trees are tracked while
  build/install/log/venv/session output remains ignored.
- Add an integrity manifest/test covering object name, required files, size,
  and content hash where runtime code already expects one.
- Commit runtime data separately and verify no file exceeds GitHub limits.

## Task 4: Add the Canonical Sorting Scene

- Port classification-bin generation from feature commit `3a203c9` into the
  stable simulator package.
- Add `classification_bins_single_random` and sorting-specific object slots to
  the canonical simulator configuration.
- Make the sorting launch explicitly enable the bin mode; ordinary launches
  create no bin.
- Validate exactly one bin, randomized placement, full initial-camera
  visibility, table support, footprint separation, base exclusion, and target
  reachability.
- Add simulator configuration and population tests before connecting motion.

## Task 5: Port Sorting and Overview-Frame Drop Targets

- Port the sorting package without compatibility wrappers.
- Make category normalization accept only `food` and `non_food`; fail closed on
  `unknown`, missing, or any other value.
- Capture one initial/home synchronized overview observation with RGB, depth,
  intrinsics, TF, timestamp, and robot pose.
- For food, locate the bin in that overview and freeze an immutable typed
  world-frame `DropTarget`; never use the post-PBVS close-up for bin location.
- For non-food, construct the typed default drop target.
- Add provenance, workspace, release-height, and stale-observation validation.
- Add focused sorting and overview-observation tests.

## Task 6: Port Tracking and PBVS onto the Stable Runtime

- Port the high-rate tracking worker, motion gate, recovery, PBVS math/node, and
  CLI entry point from the target baseline.
- Remove `_compat.py`, facade re-exports, and test-only runtime monkey-patch
  lookup paths.
- Adapt imports and ROS node ownership to the stable full workspace.
- Preserve bounded camera-relative control, workspace gates, target-jump
  rejection, target/TCP continuous-stop gates, and safe lost-target behavior.
- Use the initial overview for target selection/classification/bin localization;
  use the post-PBVS stable observation only for mask reanchor and FoundationPose.
- Add unit and ROS-independent state-machine tests.

## Task 7: Connect Stable Grasp to Explicit Classified Placement

- Use the imported stable grasp selector, planner, executor, config, and object
  policies as the sole grasp implementation.
- Remove the old `canonicalize_tabletop=False` bypass.
- Add an explicit planner input for a validated typed `DropTarget`.
- Force `execution_mode="safe_place"` in the dynamic product entry point and
  construct the drop pose using the target X/Y and validated release Z.
- Bypass pure empty-table safe-place search for the dynamic classified path.
- Rewrite `GuardedMotionExecutor` against the stable executor so it adds only
  the pre-grasp target gate and inherits all stable approach/close/lift checks.
- Prove with tests that sorting changes only the post-lift suffix.

## Task 8: Enforce the 16-Object Product Boundary

- Remove Tuna/Pudding from aliases, VLM candidates, GUI choices, simulator
  pools, scene assignment, and CLI acceptance.
- Remove retired positive Tuna/Pudding routes and tests where they no longer
  serve shared 16-object code.
- Preserve shared stable regression tests used by supported objects.
- Add a retained-test manifest, 16-object routing matrix, and negative boundary
  tests for both excluded names.

## Task 9: Integrate GUI, Browser Voice, Launch, and Dependencies

- Port the full Tkinter GUI as the product entry point.
- Use one canonical dynamic command for typed and transcribed instructions.
- Preserve browser microphone permission/capture and transcription, with no
  secret in argv, logs, or files.
- Update package entry points, package manifests, Dev Container dependency
  setup, and launch files for OpenAI, OpenCV, pycocotools, audio conversion, and
  all ROS dependencies.
- Add GUI command/status and voice server/transcription tests.
- Rewrite README with clean-clone setup, build, GUI, text, voice, and recovery
  instructions.

## Task 10: Static, Unit, and Build Verification

- Run `git diff --check`, secret scanning, file-size checks, and Python compile.
- Run retained stable tests plus new sorting/tracking/PBVS/GUI/voice tests.
- Run package-focused tests first, then the complete non-actuating suite.
- Build all ROS packages in the Dev Container with `colcon`.
- Verify installed launch files and console scripts.
- Fix regressions without weakening stable safety thresholds.

## Task 11: MuJoCo Acceptance

- Start only one simulator/MoveIt/gripper/robot-state stack.
- Verify the sorting scene contains exactly one visible bin and only supported
  target objects.
- Run a text-driven food trial with target motion and PBVS follow, then stable
  grasp and visual-bin placement.
- Run the non-food flow through the same pipeline to the default drop point.
- Exercise browser microphone capture and transcription; if the user is absent,
  complete automated audio-upload/transcription coverage and leave only the
  browser permission/speaking action as the final interactive checkpoint.
- Require release, retreat, and return home; archive logs and artifacts.

## Task 12: Delivery

- Review the final diff against the design and source baselines.
- Keep commits separated by stable import/data, simulator/sorting integration,
  tracking/stable-grasp integration, and GUI/tests/docs.
- Push the validated integration branch.
- Fast-forward local `main` only after acceptance gates pass, then push `main`
  to `https://github.com/LXnzh/VLM-grasp-sorting.git`.
- Report any remaining interactive-only voice checkpoint precisely; do not
  describe a partial run as complete E2E acceptance.

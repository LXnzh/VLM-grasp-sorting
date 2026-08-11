# VLM Tracking, Stable Grasp, and Sorting Integration Design

Date: 2026-08-11

Status: Approved in conversation

## Goal

Turn `E:\IFL\VLM_grasp_stable` into a complete, independently runnable ROS 2
workspace whose default product flow is:

1. accept a text or browser-microphone instruction in the GUI;
2. use a VLM to select one of the 16 supported YCB objects;
3. use SAM2 to lock the selected instance;
4. track the instance and follow it with bounded PBVS while it moves;
5. wait for continuous target and robot stability;
6. estimate the stable-frame object pose with FoundationPose;
7. use the object-specific grasp selection, planning, and execution behavior
   from `E:\IFL\grasp_stable_pure`;
8. classify the object as food or non-food;
9. place food in the RGB-D-localized `food_bin`, or place non-food at the
   configured default point; and
10. release, retreat, and return the robot to its initial pose.

The final GitHub remote is
`https://github.com/LXnzh/VLM-grasp-sorting.git`.

## Scope

The product supports the same 16-object release scope as the stable grasp
baseline. `tuna_fish_can` and `pudding_box` are not selectable, are not used by
random scene generation, and are not accepted by the executable pipeline.

The project includes the YCB runtime models and grasp libraries required by
the 16 supported objects. The current local data is approximately 202 MB for
the YCB dataset and 10 MB for the grasp libraries, with a largest individual
file of approximately 10.1 MB. These files fit normal Git and GitHub limits,
so the project will not add Git LFS. Third-party data sources will be credited
in the README and will not be represented as original project work.

No real-robot validation is in scope. Complete MuJoCo robot motion is in scope
and explicitly authorized.

## Repository and Workspace Structure

`E:\IFL\VLM_grasp_stable` remains the sole writable target and retains its
existing Git history. `E:\IFL\grasp_stable_pure` is a read-only source
baseline.

The reproducible import source is stable commit
`7bbfd07f76ea1c14eab61e39625654b8d4f13d3e`. The target and the new GitHub
remote share initial commit
`fb4878dbd43f83d15cd68ab1f207c9764b1c9652`; the design documentation commits
sit only on top of that common base before implementation begins.

The target will become a full ROS 2 workspace. The import includes the stable
repository's committed Dev Container configuration, scripts, ROS packages,
robot description, MoveIt configuration, MuJoCo simulator, gripper packages,
launch packages, `my_course_pkg`, and relevant tests. It excludes the stable
repository's `.git`, `build`, `install`, `log`, virtual environments, caches,
Obsidian state, Superpowers scratch state, and all uncommitted files.

The old root-level `my_course_pkg` layout and orphan root `base_env.yaml` will
be removed. ROS sources will live below `src/`, and the active simulator
configuration will remain in its canonical package location. Generated
`build`, `install`, and `log` trees stay ignored.

The stable grasp package is the code baseline. The target project's VLM,
tracking, PBVS, sorting, voice, and GUI behavior is ported onto that baseline.
Compatibility facades and legacy monkey-patch indirection are removed instead
of retained.

## Component Boundaries

### GUI and Voice Input

The Tkinter GUI remains the user-facing entry point. Text input and browser
microphone input both produce one natural-language instruction and then invoke
the same dynamic sorting command.

Browser microphone capture remains the default because it obtains microphone
permission from the Windows browser instead of requiring ALSA access from the
container. Audio is transcribed through the existing OpenAI-compatible
transcription endpoint, with `kit.whisper-large-v3` as the default model.

API keys are passed only through child-process environment variables. They are
not written into argument lists, logs, configuration files, commits, or saved
session artifacts.

The GUI starts the full simulation, launches the canonical PBVS task, streams
logs, and translates machine-readable status messages into operator guidance.
Duplicate simulator, arm interface, or gripper server launches are rejected.

### VLM, SAM2, and Tracking

The default VLM is the stable baseline's validated `azure.gpt-5-mini`. The VLM
target list contains only the 16 supported canonical names. Chinese and English
aliases map to those canonical names; an explicit object name cannot silently
be replaced by a visually similar object.

The initial synchronized RGB-D frame is saved. The VLM selects the object and
classifies it as food or non-food. SAM2 creates candidate masks and the existing
mask verifier selects the target instance. The selected canonical name,
classification, and provenance are saved in one per-run session record.

A high-rate worker maintains atomic RGB, depth, mask, timestamp, and motion
state snapshots. PBVS runs only while the target is moving or stabilizing. Its
camera-relative corrections retain bounded gain, deadband, maximum step, frame
freshness, jump rejection, and workspace constraints.

The grasp gate requires continuous target stability and TCP stability. When it
passes, the pipeline captures one synchronized stable RGB-D-mask observation,
reanchors the mask, and sends that exact observation to FoundationPose. A stale
initial frame is never reused for grasp planning.

### Stable Grasp Boundary

The following behavior comes from `grasp_stable_pure` and remains the source of
truth:

- object-specific grasp profiles and offsets;
- tabletop pose canonicalization;
- grasp-library loading and candidate expansion;
- Pear and other object-specific candidate checks;
- target bounds centering and minimum TCP height rules;
- exact scene-clearance and approach-corridor checks;
- pregrasp generation and MoveIt reachability selection;
- feedback-gated approach and vertical descent;
- gripper readiness, contact, and close validation;
- lift geometry, hold behavior, and execution failure handling; and
- return-to-initial-pose validation.

The dynamic tracking node must not pass the old
`canonicalize_tabletop=False` override. A `GuardedMotionExecutor` may subclass
the stable `ArmMotionExecutor` only to add pre-grasp target movement/loss
checks. It must not duplicate, simplify, or replace stable grasp behavior. It
will be rewritten against the current stable executor interface, without the
old compatibility wrapper.

Once grasp execution begins, camera motion caused by the robot itself must not
be interpreted as target motion. Target stability is a gate before grasp
execution; the stable executor's own feedback, planning, and gripper checks own
the actuating phase.

### Classification Placement Boundary

The initial VLM classification is stored with the selected target. Immediately
before planning, the stable synchronized RGB-D frame is used to locate the
`food_bin` when the category is food.

The sorting module returns a typed drop target containing position, category,
bin name, and provenance. Non-food returns the configured default non-food
position. Food requires a valid RGB-D-localized `food_bin`; missing or invalid
bin geometry fails before motion.

The stable planner continues to generate pregrasp, approach, close, hold, and
lift behavior. It accepts the explicit sorting drop target only for the
post-lift safe-place suffix: transfer to a high pose, descend, release, retreat,
and return home. Thus the grasp strategy is identical to the stable baseline
through reliable lift, while the requested food/non-food destination replaces
the pure product's return-to-origin suffix.

## Failure Handling

The pipeline fails closed before motion when any required dependency or
observation is unavailable or invalid, including:

- missing API credentials;
- unavailable VLM, SAM2, or FoundationPose services;
- unsupported or inconsistent target identity;
- missing, ambiguous, or invalid segmentation;
- stale or invalid synchronized RGB-D data;
- target loss or unbounded depth jumps;
- missing camera/world transforms;
- duplicate or unavailable ROS action servers;
- missing or invalid scene-clearance bounds;
- no grasp candidate passing stable selection and clearance gates; or
- food classification without a valid localized food bin.

Stable grasp thresholds are not relaxed to make a trial pass. Motion failures
use the stable executor's hold or bounded retreat behavior. A partially
executed grasp is not automatically retried. Starting a new attempt requires a
full reset, fresh perception, and fresh stability gate.

Each run stores the instruction, selected target, category, RGB, depth, mask,
FoundationPose result, plan diagnostics, status transitions, and errors in a
session directory. The GUI identifies the failed stage and directs the operator
to the saved evidence.

## Validation

Completion requires all four validation layers.

### Static Validation

- Compile all changed Python files.
- Validate dependency declarations and executable discovery.
- Check that no secret is staged.
- Check repository size and individual file sizes.
- Verify all 16 supported objects have the required YCB geometry, textures,
  and grasp libraries.
- Verify excluded objects cannot enter GUI, VLM, scene, or CLI execution.

### Automated Tests

- Preserve the complete stable non-actuating regression suite.
- Add focused tests for target aliases, the 16-object boundary, atomic tracked
  frames, movement/stability gating, PBVS bounds, target-loss handling, typed
  sorting targets, food-bin failure, non-food defaults, voice capture, GUI
  command construction, and status rendering.
- Add integration tests proving every supported object routes through stable
  grasp selection/planning and that sorting changes only the post-lift suffix.
- Verify no test relies on compatibility facades removed by the integration.

### ROS and Simulator Validation

- Rebuild the Dev Container dependency environment from declared setup.
- Build the complete ROS workspace with `colcon`.
- Verify launch files and installed executables are discoverable.
- Start MuJoCo, MoveIt, camera, gripper, and GUI.
- Verify one complete and unique robot/gripper action graph before motion.
- Verify camera frames, scene state, and clearance bounds are fresh.

### End-to-End Acceptance

Two complete MuJoCo runs are mandatory:

1. A GUI text instruction selects a food object such as
   `tomato_soup_can`. The target is moved during tracking, PBVS visibly follows,
   the target stops, stable grasp succeeds, and the object is released into the
   visually located `food_bin`.
2. Browser microphone input selects a non-food object such as `rubiks_cube`.
   The browser records and transcribes the instruction, the same dynamic
   pipeline runs, stable grasp succeeds, and the object is released at the
   default non-food destination.

Both runs must finish release, retreat, and return to the initial robot pose.
Perception-only, plan-only, close-only, or lift-only runs do not satisfy
acceptance. The user will grant browser microphone permission and speak the
voice instruction during the interactive acceptance run.

## Implementation and Git Delivery

Implementation occurs on branch
`codex/integrate-stable-tracking-sorting` in the target repository.

The target `origin` will be changed to
`https://github.com/LXnzh/VLM-grasp-sorting.git` only after verifying that the
remote `main` matches the target's current initial commit. The old GitHub remote
will not be retained or pushed.

The implementation sequence is:

1. import the committed stable ROS workspace without generated or dirty files;
2. replace the old package-only layout;
3. port and integrate VLM, tracking, PBVS, sorting, voice, and GUI modules;
4. remove excluded-object routes and obsolete compatibility layers;
5. add the 16-object YCB and grasp runtime data;
6. update dependency setup, launch files, README, and tests;
7. complete static, automated, ROS, and end-to-end validation; and
8. commit the validated result, fast-forward `main`, and push `main` to the new
   remote.

Commits should remain reviewable and separate the stable workspace/data import,
dynamic sorting integration, GUI/voice/test/documentation work, and any fixes
found by end-to-end validation.

`E:\IFL\grasp_stable_pure` remains untouched throughout the work.

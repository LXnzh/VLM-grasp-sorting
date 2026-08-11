# grasp_stable Handoff

Last updated: 2026-08-11

## Recovery Execution Status

- The approved pure-grasp recovery design is commit `b0f0160`.
- The recovery implementation plan is commit `556f65d`.
- Branch `codex/pure-grasp` has been created at `c3e7a66`.
- New worktree `E:\IFL\grasp_stable_pure` has been created successfully.
- Static verification passed: no sorting directory or sorting imports, the
  experiment-session launch exists, and the blank-input full-random prompt is
  present.
- Host-side pytest could not run because Windows exposes only the Microsoft
  Store Python application alias. Tests remain a fresh Dev Container step.
- This half-finished worktree has not yet been removed at the time of this
  archived status update.

## Read This First

This file records the current verified state of the workspace. Check it before
running more recovery commands. Update it after a blocker is resolved or the
branch/build state changes.

## Workspace and Git Layout

- Windows worktree: `E:\IFL\grasp_stable`
- Dev Container mount: `/home/ws`
- Current branch: `grasp_stable`
- Current commit: `cefd0af`
- `origin/grasp_stable`: `cefd0af`
- Relevant sorting branch: `feature/sorting` (currently `846ee5b`)
- First complete sorting integration commit: `3a203c9`
- `3a203c9` is a direct child of the current `grasp_stable` commit `cefd0af`.

This is a linked Git worktree created on Windows. Its `.git` file contains a
Windows path:

```text
gitdir: E:/IFL/PraktikumSoSe26_ML for Robotics/.git/worktrees/grasp_stable
```

The container mounts only the worktree at `/home/ws`, not the main repository's
Git metadata. Linux Git therefore interprets the Windows path incorrectly as:

```text
/home/ws/E:/IFL/PraktikumSoSe26_ML for Robotics/.git/worktrees/grasp_stable
```

Consequences:

- Run Git commands for this worktree from Windows PowerShell/Cursor, in
  `E:\IFL\grasp_stable`.
- Do not edit `/home/ws/.git`; doing so can break the Windows worktree.
- Run ROS, Python, and `colcon` commands inside the Dev Container.

## Current Uncommitted State

The last verified Windows-side `git status --short` showed:

```text
?? HOWTO.md
?? src/my_course_pkg/my_course_pkg/tasks/sorting/
```

Details:

- Root `HOWTO.md` is a newer local guide and is not tracked remotely.
- `tasks/sorting/` was restored from `feature/sorting` after the pipeline failed
  to find `my_course_pkg.tasks.sorting`.
- Do not delete or overwrite either untracked path without first checking with
  the user.

## User-Selected Product Direction

The user clarified on 2026-08-11 that the goal is **stable grasping only**.
Here, "classification" means selecting a grasp strategy/category for an
object. It does **not** mean food/non-food classification or sorting objects
into bins.

The stable implementation internally routes objects through these grasp
categories:

1. `cylindrical_can` — fixed representative: `tomato_soup_can`
2. `banana` — fixed representative: `banana`
3. `round_top` — fixed representative: `apple`
4. `box` — fixed representative: `foam_brick`
5. `tool_top` — fixed representative: `hammer`

The five fixed representatives plus one random sixth object were a category
validation milestone, not the user's final preferred runtime scene.

The final pure-grasp workflow includes interactive scene selection:

- `mix`: the fixed-priority mixed scene;
- `random`: one random object from each grasp category plus one additional
  non-duplicating object;
- `assign`: prompt for up to six required object names and randomly fill the
  remaining slots.

The user's remembered and preferred **fully random** path is `assign` with a
blank object line. The exact prompt is:

```text
Available objects (18): ...
Enter up to 6 required objects, comma-separated (blank = all random):
```

A blank response samples six unique objects from the full object pool, without
requiring one object per grasp category. The `experiment_session.launch.py`
entry point uses this interactive assigned-object flow directly.

The final release scope treated 16 objects as supported. `tuna_fish_can` and
`pudding_box` were explicitly excluded from release success criteria, even
though the simulator's full 18-object pool can still select them. Do not treat
their possible random appearance as proof that their grasp routes are release
qualified.

Do not adopt food-sorting, RGB-D bin localization, classified placement, PBVS,
or GUI integration merely to make the grasp pipeline import successfully.

## Historical Stable-Grasp Source Recovered

The previous persistent handoff is recoverable from Git object history as
`docs/agent_handoff.md` at stash commit `63b2973`. Its release checkpoint names
the coherent final stable-grasp baseline:

- `d4886e4` — generated-artifact ignores
- `242e62d` — stable multi-object grasp/session/simulator implementation
- `c3e7a66` — validated VLM default; final stable release checkpoint

That release reported:

- `632/632` non-actuating tests passed;
- all 43 changed Python files compiled and passed fatal flake8 checks;
- isolated `my_course_pkg` and `ifl_air_ur_launch` builds completed;
- interactive `mix`, category-aware `random`, and `assign` scene modes were
  implemented; blank `assign` input selects six unique full-pool objects;
- the five fixed-category representatives plus a random sixth object had also
  been validated as a category-coverage test scene.

Commit `c3e7a66` is still present locally and reachable through `refs/stash`,
but no named local or remote branch currently contains it. Preserve it before
any stash cleanup or Git garbage collection.

The historical stable release and the current `grasp_stable` branch are
different lines. They diverge after common ancestor `520d79e`:

```text
520d79e
├─ ... -> c3e7a66   historical final stable grasp
└─ ... -> cefd0af   current origin/grasp_stable, partial sorting coupling
```

At `c3e7a66`, both `perception/llm_sam2.py` and
`perception/foundationpose.py` have no `tasks.sorting` imports. Its pipeline is
the interactive grasp-perception pipeline and prompts with `Instruction:`.
Its full launch prompts for `mix/random/assign`, and its experiment-session
launch prompts for required object names with blank input meaning fully random.

## Pipeline Failure: Verified Root Cause

The current `grasp_stable` code is not a coherent pipeline revision.

At `cefd0af`:

- `perception/llm_sam2.py` imports
  `my_course_pkg.tasks.sorting.vlm_classifier`.
- `perception/foundationpose.py` imports
  `my_course_pkg.tasks.sorting.bin_locator`.
- The tracked `grasp_stable` tree does not contain `tasks/sorting/`.
- The tracked `paths.py` does not define `SORTING_OUTPUT_DIR`.

Commit `3a203c9` adds the sorting directory and its coordinated changes to:

- `my_course_pkg/paths.py`
- `perception/foundationpose.py`
- `perception/llm_sam2.py`
- grasp configuration and pick/place planning
- the MuJoCo scene and interfaces

Only restoring `tasks/sorting/` produced a mixed source tree. The current import
failure is:

```text
ImportError: cannot import name 'SORTING_OUTPUT_DIR' from 'my_course_pkg.paths'
```

The failing chain is:

```text
pipeline
  -> perception/foundationpose.py
  -> tasks/sorting/bin_locator.py
  -> paths.SORTING_OUTPUT_DIR (missing on grasp_stable)
```

This is a source-version mismatch. Rebuilding alone cannot fix it.

## Approaches Already Tried

### 1. Build only `arm_api2_py`

Command:

```bash
colcon build --packages-select arm_api2_py
```

Result: it did not repair `my_course_pkg` or the missing sorting code. Sourcing
the install also reports:

```text
not found: "/home/ws/install/arm_api2/share/arm_api2/local_setup.bash"
```

### 2. Restore only `tasks/sorting/`

The restore succeeded only when run from Windows Git. It fixed the first
`ModuleNotFoundError`, but exposed the missing `SORTING_OUTPUT_DIR`. Do not keep
copying individual dependency files one at a time.

### 3. Full `--symlink-install` build over old build artifacts

Command:

```bash
colcon build --symlink-install
```

Result: failed in `robotiq_2f_urcap_adapter` because an existing real directory
occupied a path where `ament_cmake_python` wanted to create a symlink:

```text
failed to create symbolic link ... because existing path cannot be removed:
Is a directory
```

This is consistent with switching an existing workspace from a normal build to
`--symlink-install` without first separating the old generated artifacts.

No clean full rebuild has yet been confirmed.

### 4. Python dependencies

- `pycocotools 2.0.11` was successfully installed with
  `python3 -m pip install --user pycocotools`.
- The same pip output reported that `openai` was missing.
- Installation and import of `openai` have not been independently confirmed in
  the captured terminal output.
- Do not reinstall `pycocotools` unless the Python environment changes.

## Important Documentation Mismatch

The untracked root `HOWTO.md` says `VLM_CANDIDATE_OVERRIDE` can skip VLM target
selection. In the current `grasp_stable` source, that environment variable is
used by `grasp_eval.py`, but current `llm_sam2.py` does not implement the
override. The feature/sorting version adds override handling. Do not rely on the
root HOWTO claim until the selected code revision is coherent.

## Do Not Repeat

- Do not run worktree Git commands inside `/home/ws` until the main Git metadata
  is deliberately mounted and configured for Linux.
- Do not modify `/home/ws/.git` to replace its Windows path.
- Do not repeatedly rebuild expecting it to repair missing source symbols.
- Do not restore only one more sorting-related file and continue chasing import
  errors. Select a coherent revision first.
- Do not delete `build/`, `install/`, or `log/` without explicit confirmation or
  a recoverable backup plan.
- Do not overwrite the untracked root `HOWTO.md`.

## Recovery Direction Is Resolved

Recover the coherent historical stable-grasp line represented by `c3e7a66`.
Do not complete or merge the food-sorting line merely to satisfy its imports.

Before moving the current worktree or branch, use Windows Git to create a named
recovery reference for `c3e7a66`. Preserve the current branch, the untracked
root `HOWTO.md`, and any user work. Plan the worktree transition explicitly;
do not reset, clean, or overwrite the current worktree in place.

## Build Recovery After the Source Decision

The generated workspace is currently mixed/incomplete. After choosing and
applying a coherent source revision, perform a clean build by moving the old
`build/`, `install/`, and `log/` directories to recoverable backup names, then
run one consistent build mode. Do not alternate between normal and
`--symlink-install` using the same build directory.

For the recovered stable-grasp baseline, verify imports explicitly without any
sorting dependency:

```bash
python3 -c "import openai; from pycocotools import mask; from my_course_pkg.perception.llm_sam2 import LlmSam2Node; from my_course_pkg.perception.foundationpose import FoundationPoseEstimationNode; print('stable grasp imports OK')"
```

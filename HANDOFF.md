# VLM Grasp Sorting Handoff

## Product state

The active product workspace is `E:\IFL\VLM_grasp_stable` on branch
`codex/integrate-stable-tracking-sorting`. Its remote is
`https://github.com/LXnzh/VLM-grasp-sorting.git`.

## Source-of-truth boundary

Do not redesign or independently reimplement the working dynamic chain. Use
the following two repositories as explicit, separate sources of truth:

- Dynamic GUI/VLM/SAM2/tracking/PBVS/FoundationPose/sorting chain:
  `E:\IFL\PraktikumSoSe26_ML for Robotics` at
  `98f386d9c1b3f4b6d7401b93e97c6ed263ebcf40`.
- Stable grasp selection, planning, guarded execution, and lift behavior:
  `E:\IFL\grasp_stable_pure` runtime commit
  `c3e7a66f51ce5ff9f771a9de4a8b58ef2d986ae4`.

The feature repository's `grasp/` implementation is smaller than the stable
runtime and is not a replacement for it. The current grasp audit found only
the intended product differences from the stable runtime: removal of the
unsupported Tuna/Pudding paths, formatting/type cleanup, and the classified
sorting placement suffix. Keep that boundary. When a cross-component call
fails, compare the caller and callee contracts against the repositories above
before writing new orchestration.

The implemented product path is:

1. GUI text or browser-microphone instruction;
2. VLM object selection and strict `food` / `non_food` classification;
3. overview RGB-D food-bin localization and immutable drop-target capture;
4. SAM2 mask initialization, tracking, and PBVS camera motion;
5. stable RGB-D capture and FoundationPose pose estimation;
6. the `grasp_stable_pure` grasp-selection, planning, and guarded execution
   prefix through grasp and lift;
7. safe-place sorting into the visual food bin or the configured non-food
   location.

Unknown identities, invalid categories, missing depth, missing bin evidence,
tracking loss, pose-estimation failure, planning failure, and execution failure
all stop before an unsafe downstream action.

## Supported objects

The release scene, GUI/VLM catalog, CLI assignment, FoundationPose catalog,
and grasp assets use the same 16 objects:

`tomato_soup_can`, `gelatin_box`, `banana`, `apple`, `lemon`, `peach`, `pear`,
`orange`, `plum`, `sponge`, `hammer`, `baseball`, `tennis_ball`, `racquetball`,
`foam_brick`, and `rubiks_cube`.

`tuna_fish_can` and `pudding_box` are deliberately unsupported. Their
specialized runtime, calibration data, tools, and positive tests were removed;
negative boundary tests verify that both names are rejected.

## Runtime entry points

Follow `README.md` for setup. The normal interactive launch keeps the MuJoCo
viewer enabled. Display-less validation uses `sim_headless:=true`, which also
selects EGL for offscreen RGB-D rendering.

The repeatable local stack check is:

```bash
bash scripts/smoke_test_full_stack.bash
```

It waits for the real simulator, camera threads, MoveIt, and gripper server,
then sends a gripper command to verify delayed adapter reconnection. External
VLM, SAM2, and FoundationPose services still require the endpoints and API key
documented in `README.md`.

## Acceptance evidence

Verified on 2026-08-11 in the project ROS 2 Humble container:

- `my_course_pkg`: 436 functional tests passed, 3 linter wrappers excluded;
- MuJoCo simulator: 89 tests passed, including rendered RGB-D bin localization;
- launch and delayed gripper reconnect: 53 tests passed;
- all 10 ROS packages built successfully with `--symlink-install`;
- installed `pbvs_sorting_grasp --help`, launch `--show-args`, and GUI import
  passed;
- changed-path fatal Flake8, full Flake8 for all new product-path files,
  `compileall`, Bash syntax, and `git diff --check` passed;
- `scripts/smoke_test_full_stack.bash` passed with EGL RGB-D rendering,
  MoveIt, the simulator socket, and a real delayed gripper reconnect/activate.

The legacy ament Flake8 wrapper is not used as a release gate because it scans
generated `build/` output and unrelated inherited packages. Product changes
are instead checked directly before every handoff.

## 2026-08-11 FoundationPose handoff-contract failure

The first GUI launch problem was WSLg (`rdp_peer is not initialized`), not a
ROS build hang. Restarting WSL restored the GUI. The subsequent banana task
entered the real dynamic chain, selected/reanchored the banana mask, and then
failed before the FoundationPose request, planning, or robot motion with:

```text
AttributeError: '_DirectMaskFoundationPose' object has no attribute
'_validate_mask_geometry'
```

The complete preserved session log is
`logs/gui_ros_log_20260811_214253.txt`; the traceback is near lines 1266-1280.
Rebuilding unchanged sources could never fix this failure.

Root cause: integration commit `0e4f484` copied the reference chain's
`_DirectMaskFoundationPose._load_mask()` call to `_validate_mask_geometry()`
but omitted the corresponding FoundationPose constructor configuration and
method. Existing sorting tests mocked the FoundationPose boundary and
therefore did not instantiate the broken concrete subclass.

The repair mechanically restores the reference contract in
`perception/foundationpose.py`, including the 5 px border and 200 px minimum
area defaults, validates every candidate before ranking, and adds both the
reference geometry tests and a concrete `_DirectMaskFoundationPose` service-
boundary regression test. Verified after the repair:

- focused FoundationPose/sorting tests: 36 passed;
- complete `my_course_pkg` functional suite: 435 passed;
- changed files: `compileall`, fatal-level Flake8, and `git diff --check`
  passed;
- `colcon build --symlink-install --packages-select my_course_pkg` passed;
- the live ROS graph still exposed MuJoCo, MoveIt, robot-state publisher,
  gripper adapter, and watchdog nodes with no stale grasp child process.

A post-repair GUI banana motion trial is still required before calling this
specific runtime incident fully accepted. Record its saved GUI log and result
here; do not substitute mocked tests for that final evidence.

## 2026-08-11 guarded-executor handoff-contract failure

The next real banana trial proved that the repaired chain passed VLM selection,
food classification, RGB-D food-bin localization, SAM2, stable tracking,
FoundationPose, grasp planning, gripper opening, and the MoveIt pre-grasp move.
It then exited at the linear approach with:

```text
TypeError: ArmMotionExecutor.move_to_pose() got an unexpected keyword argument
'avg_speed'
```

The preserved log is `logs/gui_ros_log_20260811_220836.txt`; the successful
upstream evidence is near lines 8948-9206 and the traceback is near lines
9408-9428. The MuJoCo viewer merely retained the last commanded pre-grasp
pose, so the apparently frozen viewer did not mean computation was active.

Root cause: `tasks/tracking/guarded_executor.py` was integrated with the old
course executor's optional `avg_speed` interface, while this product correctly
retains the stable-grasp executor contract from `grasp_stable_pure`, whose
`move_to_pose()` accepts only the pose and applies its own verified speed and
filter constants. The repair removes the obsolete parameter from the dynamic
guard adapter instead of modifying the stable core. A regression test now
calls `GuardedMotionExecutor.move_to_pose()` through the real stable base class
and verifies the motion guard plus the stable interpolation settings.

After this repair, the complete `my_course_pkg` functional suite passed 436
tests and the package rebuilt successfully. The saved follow-up trial
`logs/gui_ros_log_20260811_221617.txt` contains a successful banana close,
lift, classified transfer, release, retreat, and return command. The same log
also contains an interleaved failed run caused by Reset/Start/Start launching
overlapping work; judge the successful task by its complete state sequence and
do not start a second task while one is active.

## 2026-08-12 scene settling and tracking cue

The six-object `scene_mode=random` composition and fixed placement-slot order
remain unchanged. It still selects the configured stratified categories,
including the singleton banana and hammer pools, plus one remaining object.

The hammer launch was caused by mesh placement using the visual OBJ bottom
even though MuJoCo contacts use the VHACD collision meshes. For the hammer the
collision mesh extends slightly below the visual mesh, so it could start in
the table and be expelled by the contact solver. Mesh placement now:

- retains the rotated visual-mesh centroid for the existing XY layout;
- uses the minimum rotated Z across every collision mesh for table contact;
- leaves the collision bottom 0.5 mm above the table;
- applies this single rule to all mesh objects, with no hammer-specific path.

PBVS now declares a 10-second motion-observation default. The GUI has a
full-width operator banner above the ROS log. `TARGET_LOCKED` and
`PBVS_OBSERVING_FOR_MOTION` show the explicit MuJoCo
Ctrl+Shift+right-drag instruction; an immediately following stable-gate line
cannot overwrite that prompt while the drag window is active. Follow, stop,
freeze, execution, failure, and completion states replace the banner with the
corresponding instruction.

Verified in the project container:

- focused placement/PBVS/GUI tests: 53 passed;
- complete MuJoCo simulator suite: 91 passed, including unchanged stratified
  six-object selection and a real YCB hammer settling test;
- complete `my_course_pkg` functional suite: 443 passed with the three legacy
  ament linter wrappers excluded as documented above;
- the real hammer collision bottom is 0.5 mm above the table at spawn and its
  one-second headless settle stays within 5 mm upward and horizontal motion;
- changed-file `compileall`, fatal Flake8, and `git diff --check` passed;
- `colcon build --symlink-install --packages-select my_course_pkg` passed.

At handoff time the visible GUI/simulator were deliberately not force-restarted:
the API key existed only in GUI memory (or was absent from the environment),
so killing the old process would discard it. Close the old GUI, rerun
`python3 gui_manager.py`, and start a new scene before visually judging either
the banner or the corrected hammer spawn.

## 2026-08-12 GUI-owned process-tree shutdown

Closing the old GUI left its `ros2 launch` process alive because
`gui_manager.py` destroyed only Tk and did not terminate descendants. A new
GUI then started a second full stack because its in-memory `active_processes`
registry could not see the orphan. The ROS graph contained duplicate
`move_group`, `moveit2_iface`, robot-state publisher, watchdog, and gripper
nodes. Action clients reported `There may be more than one action server`, and
goal/result responses crossed between the two stacks.

The old orphan was process group 2830 with launch PID 3242. It and all of its
children were terminated; a refreshed ROS graph confirmed one remaining set
of control nodes.

The GUI now owns complete process trees:

- every `Popen` command starts a new Linux session;
- main-window close and `mainloop()` exit terminate all registered process
  groups with SIGTERM, one bounded wait, SIGKILL fallback, and root reaping;
- cleanup is idempotent and refuses new commands after closing begins;
- before scene launch, `/proc` is scanned for the exact full-stack launch
  marker; an externally owned or orphan stack blocks a second launch instead
  of being killed automatically.

Verification:

- GUI lifecycle and guidance tests: 13 passed;
- complete `my_course_pkg` functional suite: 450 passed with the three legacy
  ament linter wrappers excluded;
- changed-file `compileall`, fatal Flake8, and `git diff --check` passed;
- the live guard detected the one remaining launch as PID 23622;
- the live ROS graph contained only one `move_group`, `moveit2_iface`,
  robot-state publisher, watchdog, and gripper adapter.

The visible GUI at implementation time was PID 23534 and had loaded the old
source before this repair. Its launch PID 23622 remained active, although its
MuJoCo process had already exited. Do not start another scene from a second
GUI. When the API key can be re-entered, terminate that old GUI process group,
start `python3 gui_manager.py` again, and use the new GUI for final close-path
runtime acceptance.

## 2026-08-12 global vertical Z tolerance

A moved `gelatin_box` trial completed VLM selection, PBVS tracking,
FoundationPose, planning, pregrasp calibration, and the first 18 of 20 vertical
descent waypoints. Waypoint 19 stopped before gripper close with 0.3 mm XY
error and a +5.2 mm Z residual because the global vertical Z tolerance was
3 mm. The MuJoCo view showed both open fingertips already straddling the box.

The user explicitly selected the smallest configuration change and accepted
that the executor will still command waypoint 20 rather than close early at
waypoint 19. The default `GRASP_VERTICAL_APPROACH_Z_TOLERANCE_M` is now 6 mm.
This is the existing global symmetric vertical tolerance, so it applies to
pregrasp, every descent waypoint, and ordinary final verification in both Z
directions. XY gates, waypoint spacing, timeout, bounds logic, commands,
gripper behavior, and non-vertical profiles are unchanged.

Verification in the project container:

- focused configuration and guarded-executor tests: 168 passed;
- complete `my_course_pkg` functional suite: 454 passed with the three legacy
  ament linter wrappers excluded;
- positive and negative 5.2 mm waypoint residuals pass, while positive and
  negative 6.1 mm residuals hold and fail;
- changed-file `compileall`, fatal Flake8, and `git diff --check` passed;
- `colcon build --symlink-install --packages-select my_course_pkg` passed;
- the installed runtime reports the default vertical Z tolerance as `0.006`.

No robot motion was launched automatically. The next `gelatin_box` trial
should show waypoint 19 passing at +5.2 mm, followed by the unchanged final
waypoint and final verification. A later residual above 6 mm must still hold
and abort.

## 2026-08-12 GitHub branch and language publish

The integration branch is now named `integrate-stable-tracking-sorting`
locally and on `origin`. The old remote
`codex/integrate-stable-tracking-sorting` branch was deleted after the new
remote ref was verified against local HEAD. The unrelated `.obsidian/`
directory remains untracked.

GitHub's Jupyter Notebook language count came entirely from
`src/ifl_air_mujoco_sim/examples/test_render.ipynb`. The file contains about
3.6 KB of cell source and 3.43 MB of generated execution output. A
path-specific `.gitattributes` rule now marks only this file as
`linguist-generated=true`; the Notebook content and outputs are unchanged.
`git check-attr` confirms the rule. After the change reaches `main`, Python is
expected to represent about 80% of the remaining recognized source bytes.

Draft PR #2 publishes the complete integration branch into `main`:
`https://github.com/LXnzh/VLM-grasp-sorting/pull/2`. At creation GitHub
reported the PR as mergeable with a clean merge state. The repository homepage
will continue to show the old language totals until the PR is merged and
GitHub recalculates the default branch.

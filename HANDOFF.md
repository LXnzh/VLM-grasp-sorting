# VLM Grasp Sorting Handoff

## Product state

The active product workspace is `E:\IFL\VLM_grasp_stable` on branch
`codex/integrate-stable-tracking-sorting`. Its remote is
`https://github.com/LXnzh/VLM-grasp-sorting.git`.

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

- `my_course_pkg`: 431 functional tests passed, 3 linter wrappers deselected;
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

# Numbered-YCB Runtime Restoration Implementation Plan

## Objective

Restore the `c3e7a66`-based pure-grasp runtime against the recovered numbered
YCB model directories and grasp archives. Reuse only the YCB loading concept
from `56c2c25`; preserve the current scene-selection, perception, grasp,
planning, and execution behavior.

## Step 1: Protect Current State and Validate Inputs

- Preserve the existing uncommitted `HANDOFF.md` addition and `.obsidian/`.
- Confirm `E:\YCB_data\YCB` contains the expected 18 numbered object
  directories.
- For every directory, require `textured.obj`, `textured.mtl`,
  `texture_map.png`, and at least one `textured_vhacd_collision_*.obj`.
- Confirm `E:\YCB_data\grasps` contains 18 directories, 906 `.npz` files,
  and 10,019,425 bytes.
- Confirm the Dev Container still mounts this worktree at `/home/ws`.

Stop before code changes if a source dataset check fails.

## Step 2: Install Local Runtime Data

- Create the exact destination directories only when absent.
- Copy the contents of `E:\YCB_data\YCB` into
  `src\my_course_pkg\YCB_Dataset\ycb` without an extra nesting level.
- Copy the contents of `E:\YCB_data\grasps` into workspace-root `grasps`
  without an extra nesting level.
- Compare source and destination relative file paths, sizes, and counts.
- Confirm Git excludes both local data destinations.

Do not remove or overwrite unrelated destination files. If a destination is
already populated inconsistently, stop and diagnose it.

## Step 3: Define the Numbered-YCB Scene Contract

- Replace mesh-object `xml_path` entries in `base_env.yaml` with explicit
  `ycb_dir` entries for the numbered directories.
- Keep all friendly object names, scene categories, positions, orientations,
  and selection behavior unchanged.
- Add focused configuration coverage proving all 18 paths are unique,
  numbered, and present in the local dataset.

## Step 4: Implement the MuJoCo Loader

- Refactor mesh placement to read `<ycb_dir>/textured.obj` directly.
- Add a small validator returning the required visual, material, texture, and
  sorted collision paths for one selected object.
- Add a focused builder that creates the object's MuJoCo assets, material,
  visual geom, collision geoms, and free joint in memory.
- Preserve the historical loader's visual/collision groups, friction, and
  first-collision `mass="0.1"` values.
- Replace XML parsing in the object insertion loop with the new builder.
- Fail with object- and path-specific messages for missing assets.

Do not retain `xml_path` support and do not copy unrelated changes from
`56c2c25`.

## Step 5: Resolve Numbered Meshes in FoundationPose

- Update the production `NAME2MESH` values to the 18 numbered directories.
- Update `foundationpose_verify.py` to use the same numbered mapping behavior.
- Preserve current target canonicalization, mask selection, bundle format,
  service retry policy, and output handling.
- Add tests for underscore and space aliases and for missing numbered meshes.

## Step 6: Run Focused Tests and Static Checks

Inside the Dev Container:

- compile every changed Python file;
- run new numbered-YCB simulator tests;
- run existing scene-selection and scene-clearance tests;
- run FoundationPose mask/path tests;
- run the pure-grasp selector, planner, executor, and trajectory regressions
  identified in `HANDOFF.md`;
- run fatal flake8 checks on changed Python files when the repository's
  configured tooling is available.

No command in this step starts a ROS graph or issues motion.

## Step 7: Build and Perform No-Motion Runtime Gates

- Run the full existing `colcon build --symlink-install` workspace build.
- Source the result and import `my_course_pkg.pipeline`.
- Compile representative one-object and six-object scenes through MuJoCo
  directly.
- Launch `experiment_session.launch.py`, submit the API key prompt as needed,
  leave the object selection blank, and require six unique selected objects.
- Confirm the simulator remains alive without an XML/mesh error, then stop the
  session before grasp execution.

## Step 8: Record and Commit the Result

- Update the existing dirty `HANDOFF.md` in place, preserving its prior
  restoration note.
- Record data locations, loader provenance, checks performed, and remaining
  live-motion boundary.
- Review the complete diff to ensure no sorting, grasp tuning, generated data,
  `.obsidian/`, or unrelated file is staged.
- Commit the implementation, tests, configuration, and handoff update.

## Acceptance Gate

The work is complete only when all 18 numbered objects resolve consistently in
the simulator and FoundationPose, the grasp library matches the recovered
source exactly, focused regressions and the full build pass, and blank
interactive input starts a live six-object simulation without issuing grasp
motion.

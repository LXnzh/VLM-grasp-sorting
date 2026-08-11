# Numbered-YCB Runtime Restoration Design

## Context

The pure-grasp worktree is based on the mature grasp release `c3e7a66` and
must remain the stable grasp product. Its current simulator configuration
expects one hand-authored MuJoCo XML file per YCB object, for example
`YCB_Dataset/ycb/hammer.xml`. The recovered YCB library instead uses the
standard numbered directory layout:

```text
005_tomato_soup_can/
007_tuna_fish_can/
...
077_rubiks_cube/
```

Each directory contains `textured.obj`, `textured.mtl`, `texture_map.png`,
`info.yml`, and decomposed `textured_vhacd_collision_*.obj` meshes. It does
not contain MuJoCo XML files. This mismatch causes the simulator process to
exit before the random scene is created.

Commit `56c2c25` proves that this exact numbered layout was previously loaded
successfully by synthesizing the required MuJoCo elements in memory. That
commit is not the final stable-grasp release, so it must be used only as the
reference for the YCB loader. Its older grasp selector, executor, launch, and
grasp-tuning changes must not be merged.

## Goal

Restore the complete pure-grasp runtime by teaching the `c3e7a66`-based code
to consume the recovered numbered YCB data directly, while preserving all
current stable grasp behavior and the interactive blank-input six-object
random workflow.

## Local Data Contract

The recovered data remains local runtime data and is not committed to Git.
The worktree's existing Git exclude rules already cover both destinations.

Copy the contents of each source directory without adding another `YCB` or
`grasps` nesting level and without renaming its numbered directories:

```text
E:\YCB_data\YCB
  -> E:\IFL\grasp_stable_pure\src\my_course_pkg\YCB_Dataset\ycb
  -> /home/ws/src/my_course_pkg/YCB_Dataset/ycb

E:\YCB_data\grasps
  -> E:\IFL\grasp_stable_pure\grasps
  -> /home/ws/grasps
```

The grasp source contains 18 directories, 906 `.npz` files, and exactly
10,019,425 bytes. The destination must match those values after copying.

The numbered YCB directory is the only supported object-model layout for this
product. Do not generate 18 XML files, create unnumbered duplicate folders, or
retain the obsolete `xml_path` configuration as a compatibility fallback.

## Runtime Design

### Scene configuration

Replace every mesh object's `xml_path` in `base_env.yaml` with an explicit
`ycb_dir` pointing to its numbered directory. Friendly runtime names such as
`hammer` and `tomato_soup_can` remain unchanged; only their asset locations
change.

This keeps scene selection independent from storage naming. The current
`mix`, category-aware `random`, and interactive `assign` selection logic is
unchanged.

### Simulator object loader

Adapt only the numbered-directory loading behavior demonstrated by
`56c2c25` to the current `populate_scene.py`:

1. Validate the selected object's `ycb_dir` and required visual/collision
   assets.
2. Read `textured.obj` for the existing mesh-centering and table-placement
   calculation.
3. Build the MuJoCo `<asset>` and `<body>` elements in memory.
4. Use `texture_map.png` and `textured.obj` for a non-colliding visual geom.
5. Use every sorted `textured_vhacd_collision_*.obj` as a collision geom.
6. Add one free joint and preserve the historical loader's simulator physics:
   friction `0.9 0.2 0.05`, with `mass="0.1"` on the first collision geom
   exactly as in `56c2c25`.
7. Merge the generated elements into the current scene exactly where parsed
   XML content was previously merged.

Missing collision meshes are an error. The loader will not silently replace
them with the visual mesh, because all 18 recovered objects include decomposed
collision assets and a silent fallback could materially change contact
behavior.

The current scene-clearance calculation and later stable-grasp changes remain
authoritative. The old `56c2c25` implementation is adapted to those current
interfaces rather than copied wholesale.

### FoundationPose mesh resolution

Update the production FoundationPose name-to-mesh mapping and its verification
utility so friendly object names resolve to numbered directories. Both paths
continue to package `textured.obj`, `textured.mtl`, and the texture referenced
by that material for the FoundationPose service.

No segmentation, VLM, pose-estimation, grasp-selection, planning, execution,
or release thresholds change.

## Error Handling

For a selected object, fail before MuJoCo compilation or an HTTP request when
any required asset is missing. The exception must identify:

- the friendly object name;
- the numbered directory that was inspected; and
- the missing file or collision-file pattern.

Unknown scene object names continue to use the current scene-selection
validation. There is no automatic search, renaming, download, XML generation,
or alternate dataset layout.

## Verification

Verification proceeds without robot motion:

1. Confirm destination data integrity: all 18 numbered YCB directories, the
   required visual/material/texture assets and collision meshes, plus the
   exact recovered grasp-library directory/file/byte counts.
2. Add focused simulator tests for numbered-directory validation, in-memory
   MuJoCo element construction, collision ordering, mesh placement, and clear
   missing-asset failures.
3. Add focused FoundationPose tests proving every supported friendly name and
   space-separated alias resolves to the correct numbered directory.
4. Run existing scene-selection and scene-clearance tests to prove `mix`,
   category-aware `random`, and blank-input `assign` behavior did not change.
5. Compile changed Python files and run the relevant pure-grasp perception,
   selector, planner, and executor regressions.
6. Build the full ROS workspace using the existing `--symlink-install` build
   mode and verify `my_course_pkg.pipeline` imports.
7. Load a representative object and a six-object scene through MuJoCo without
   starting ROS motion.
8. Launch `experiment_session.launch.py`, leave the object prompt blank, and
   confirm six unique objects are selected and the simulator remains alive
   without any missing-XML or missing-mesh error. Stop before grasp execution.

## Documentation and Provenance

After successful verification, update `HANDOFF.md` with:

- the local YCB and grasp data destinations;
- the distinction between `56c2c25` as loader provenance and `c3e7a66` as the
  stable-grasp baseline;
- the exact tests/build/runtime checks completed; and
- the authoritative interactive launch command.

The implementation commit must remain narrowly scoped to YCB data resolution,
the simulator loader, FoundationPose mapping, tests, and the handoff update.

## Out of Scope

- switching or resetting the branch to `56c2c25`;
- merging the complete `56c2c25` commit;
- changing grasp candidates, thresholds, planner, executor, or object-specific
  tuning;
- food sorting, bin localization, PBVS, tracking, or GUI dependencies;
- committing YCB meshes or grasp archives;
- generating or maintaining per-object MuJoCo XML files;
- live robot or simulated grasp motion during restoration verification.

## Acceptance Criteria

- The branch retains the `c3e7a66` stable-grasp lineage and contains none of
  the old commit's unrelated grasp behavior.
- All 18 recovered numbered YCB directories are consumed directly.
- `/home/ws/grasps` exactly matches the recovered 18-directory, 906-file,
  10,019,425-byte library.
- Simulator and FoundationPose resolve the same numbered directory for every
  supported object.
- No runtime code requires `<object>.xml` or an unnumbered YCB directory.
- Existing pure-grasp regression tests and the full workspace build pass.
- Blank interactive input creates six unique full-pool objects and the
  simulator remains running.
- No sorting dependency or grasp tuning is introduced, and no robot motion is
  issued during verification.

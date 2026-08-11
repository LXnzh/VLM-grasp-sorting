# Pure Stable-Grasp Handoff

Last updated: 2026-08-11

## Recovery Completion Status

- Recovery completed successfully on 2026-08-11.
- The prior half-finished worktree `E:\IFL\grasp_stable` was unregistered and
  removed after this worktree passed static verification.
- On 2026-08-11 the user asked to restore that folder. It was recreated as a
  linked worktree at `E:\IFL\grasp_stable` on branch `grasp_stable`
  (`556f65d`, two commits ahead of `origin/grasp_stable` at `cefd0af`).
  Untracked `HOWTO.md` and `tasks/sorting/` were restored from the recovery
  archive and `feature/sorting`. The original deleted grasp-library Recycle Bin
  payload remains unavailable; equivalent recovered data from
  `E:\YCB_data\grasps` is now installed only in this pure worktree. Do not treat
  the restored half-finished worktree as the pure-grasp product; continue here.
- Its Dev Container `keen_darwin` was stopped because it still mounted the old
  path. Do not restart that obsolete container.
- The sorting worktree remained at commit `98f386d` with the same pre-existing
  untracked files before and after recovery.
- This branch's documentation commit is `a30de4e`, whose direct parent is the
  pure-grasp runtime baseline `c3e7a66`.
- A post-recovery diff confirmed that runtime source remains identical to
  `c3e7a66`.

## Product Boundary

This worktree is the pure stable-grasp product. It performs scene generation,
perception, grasp-strategy selection, guarded grasp execution, lift, and the
configured pure-grasp completion path.

It must not depend on:

- food/non-food classification;
- RGB-D sorting-bin localization;
- classified placement;
- `my_course_pkg.tasks.sorting`;
- the GUI/tracking/PBVS product line.

The separate sorting worktree is retained at:

```text
E:\IFL\PraktikumSoSe26_ML for Robotics
branch: feature/gui-pipeline
```

Do not modify that worktree while working here unless the user explicitly
places it in scope.

## Git Identity and Provenance

- Windows path: `E:\IFL\grasp_stable_pure`
- Dev Container path after reopening: `/home/ws`
- Branch: `pure-grasp`
- Runtime baseline: `c3e7a66`
- Baseline subject: `fix: use validated VLM default`
- Baseline date: 2026-07-19

The documentation commit above that baseline must contain documentation only.
Runtime files should remain identical to `c3e7a66` until a separately reviewed
pure-grasp change is requested.

The July release record reported 632/632 non-actuating tests passing, 43
changed Python files compiling and passing fatal flake8, and successful
isolated builds of `my_course_pkg` and `ifl_air_ur_launch`.

## Interactive Fully Random Scene

The remembered user workflow is the experiment-session object prompt:

```text
Available objects (16): ...
Enter up to 6 required objects, comma-separated (blank = all random):
```

Behavior:

- enter one to six names to keep those objects first and randomly fill the
  remaining slots;
- press Enter on a blank line to sample six unique objects from the configured
  16-object pool.

This blank `assign` behavior is the fully random scene. Do not confuse it with
the separately named `random` scene mode, which samples one object from each
grasp category and then one additional unique object.

The full launch supports `mix`, category-aware `random`, and `assign`. The
experiment-session launch uses the interactive assigned-object workflow.

## Grasp Categories

After an object is generated, the grasp selector routes it through the
appropriate strategy:

- `cylindrical_can`
- `banana`
- `round_top`
- `box`
- `tool_top`

The category system selects grasp behavior; it does not constrain the blank
fully random scene.

The release success scope contains 16 objects. `tuna_fish_can` and
`pudding_box` are excluded from scene generation as well as from the release
success criteria.

## Sixteen-Object Scene Pool

On 2026-08-11 the scene-generation pool was reduced from 18 objects to the 16
release-qualified objects. `tuna_fish_can` and `pudding_box` were removed from
both the simulator `objects` list and the category-aware random pools.

The interactive prompt, blank and partial `assign`, `random`, and `mix` now all
use the same reduced pool. Explicit assignment of either excluded name fails
the existing unknown-object validation. The downstream grasp implementations,
YCB mappings, FoundationPose support, grasp data, and local model assets remain
unchanged.

Verification completed without launching the simulator or issuing perception
or grasp commands:

- all MuJoCo simulator tests: `85/85`;
- all full-launch and experiment-session tests: `44/44`;
- downstream YCB mapping tests: `25/25`;
- changed Python compilation and fatal flake8 checks passed;
- `git diff --check` passed, and the active scene YAML contains neither
  excluded name.

## Racquetball Instruction Alias Expansion

On 2026-08-11 the existing strict racquetball perception alias was expanded to
recognize `racquetball` and `blue ball` as well as `blue racquetball`. All
three singular, space-separated phrases select canonical `racquetball`, send
`blue ball.` to GroundingDINO/SAM2, accept only the `blue ball` SAM2 class, and
retain the visual-verifier hint that rejects a yellow-green tennis ball.

The existing fail-closed alias behavior remains unchanged: an ambiguous,
invalid, or failed VLM mask-verifier decision cannot use the highest detection
score as a fallback. No FoundationPose, grasp-planning, scene-clearance, or
motion threshold changed.

Non-actuating verification completed in the Dev Container:

- target-alias, mask-matching, and VLM-configuration tests: `36/36`;
- changed Python compilation and fatal flake8 checks passed;
- `my_course_pkg` rebuilt successfully with `--symlink-install`;
- the installed resolver maps all three approved instructions to canonical
  `racquetball` with grounding prompt `blue ball`.

No ROS graph, simulator, perception service, or robot motion ran during this
change. The next runtime check should run a fresh perception pipeline and
confirm the automatically selected mask covers the blue racquetball before
any plan-only or motion trial.

## Verified During Recovery

Before the prior worktree was removed, this worktree was verified to have:

- branch `pure-grasp` rooted at `c3e7a66`;
- no `tasks/sorting/` directory;
- zero Python references to `tasks.sorting`, `SORTING_OUTPUT_DIR`,
  `classify_food`, or `bin_locator`;
- `experiment_session.launch.py` present;
- the exact `blank = all random` prompt present;
- separate category-aware random and full-pool assigned-object selectors;
- no runtime diff from `c3e7a66`.

Host-side pytest was not available because Windows exposed only the Microsoft
Store Python application alias. Container tests remain required.

## Preserved Documentation

- `docs/agent_handoff.md`: detailed historical July stable-grasp handoff.
- `docs/recovery/2026-08-11-half-finished-HANDOFF.md`: diagnosis and recovery
  record from the removed August half-finished worktree.
- `docs/recovery/2026-08-11-half-finished-HOWTO.md`: archived run guide from
  that worktree; it is not authoritative for this recovered version.
- `docs/superpowers/specs/2026-08-11-pure-grasp-worktree-recovery-design.md`:
  approved recovery design.
- `docs/superpowers/plans/2026-08-11-pure-grasp-worktree-recovery.md`:
  executed recovery plan.
- `docs/superpowers/specs/2026-08-11-numbered-ycb-runtime-restoration-design.md`:
  approved numbered-directory runtime design.
- `docs/superpowers/plans/2026-08-11-numbered-ycb-runtime-restoration.md`:
  executed runtime-restoration plan.
- `docs/superpowers/specs/2026-08-11-sixteen-object-scene-pool-design.md`:
  approved design for the current 16-object scene pool.

## Numbered YCB Runtime Restoration

The XML startup blocker was resolved on 2026-08-11. The recovered object data
uses the standard 18 numbered YCB directories and does not contain hand-authored
MuJoCo object XML files. Each retained scene object declares `ycb_dir`; the
simulator validates its directory and generates its visual, collision, free
joint, texture, and material elements in memory. Scene-clearance bounds read
the same visual and sorted collision meshes directly. No object-level
`xml_path` fallback remains. The two objects later excluded from scene
generation retain their downstream mappings and local data.

Commit `56c2c25` is loader provenance only. Its in-memory numbered-directory
concept and historical friction/mass values were adapted to this branch; none
of its older selector, executor, launch, or grasp-tuning behavior was merged.
The stable-grasp lineage remains based on `c3e7a66`.

Local runtime data is installed at:

```text
/home/ws/src/my_course_pkg/YCB_Dataset/ycb
/home/ws/grasps
```

The YCB destination matches `E:\YCB_data\YCB` by all 177 relative paths and
sizes. All configured simulator scene names and all retained downstream YCB
mappings resolve to the corresponding numbered directory. The grasp
destination matches `E:\YCB_data\grasps` by all
906 `.npz` relative paths and sizes: 18 directories and exactly 10,019,425
bytes. Both destinations remain local Git-excluded runtime data.

Verification completed without perception or grasp motion:

- numbered-YCB, scene, clearance, launch, and experiment-session tests:
  `121/121`;
- FoundationPose mapping plus selector/planner/executor/trajectory regression:
  `353/353`;
- changed Python compilation and fatal flake8 checks passed;
- at the 18-object recovery baseline, all simulator geometry caches and
  FoundationPose mappings loaded;
- representative one-object and blank-assignment six-object MuJoCo models
  compiled headlessly with complete clearance bounds;
- all 10 ROS packages rebuilt successfully with `--symlink-install`, and the
  installed `my_course_pkg.pipeline` import passed;
- an interactive blank-input smoke selected six unique objects (`hammer`,
  `foam_brick`, `peach`, `tomato_soup_can`, `rubiks_cube`, `pudding_box`),
  launched the viewer and renderer, created both reset services, and completed
  the 100-sample six-object clearance profiler before the deliberate SIGINT.

The deliberate smoke-test shutdown produced ordinary ROS/GL interruption
tracebacks after SIGINT; the MuJoCo process itself finished cleanly. No VLM,
SAM2, FoundationPose, planner, trajectory, arm, or gripper command ran.

## Container and Build Status

The fresh Dev Container was prepared and the complete workspace was built with
`--symlink-install` on 2026-08-11. All 10 packages finished successfully.

- The MuJoCo package-local virtual environment was created from
  `src/ifl_air_mujoco_sim/requirements.txt`.
- ROS-side `openai` and `pycocotools` imports pass under `/usr/bin/python3`.
- Importing `my_course_pkg.pipeline` succeeds.
- `arm_api2` now explicitly declares and links `yaml-cpp`; this fixed the
  `undefined reference to YAML::...` linker failure.

The interactive entry point is:

```bash
ros2 launch ifl_air_ur_launch experiment_session.launch.py
```

It prompts first for the hidden VLM API key and then for up to six object names.
Blank object input selects six unique objects from the 16-object pool.

The numbered-YCB and grasp-library startup blockers are resolved. The next run
may use the authoritative interactive command above. Stop before live or
simulated grasp motion unless the user explicitly authorizes it; after any
reset, require fresh perception before `grasp_demo`.

## Operational Pitfalls

- Run worktree Git commands from Windows, not `/home/ws` in the container.
- Do not reintroduce the August partial sorting directory to repair imports;
  this baseline has no sorting imports.
- After `/reset_sim`, rerun perception before grasping because old perception
  output is stale.
- Do not run two Dev Containers/ROS graphs for these worktrees simultaneously
  with host networking.
- Do not reuse build, install, log, Python cache, or experiment artifacts from
  another worktree.
- Read the detailed historical handoff before changing grasp safety thresholds
  or object-specific behavior.

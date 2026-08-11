# Pure-Grasp Worktree Recovery Design

## Context

The local repository must retain two independent products:

1. the existing GUI/perception/sorting workspace at
   `E:\IFL\PraktikumSoSe26_ML for Robotics`; and
2. a pure stable-grasp workspace recovered from the July release history.

The current linked worktree at `E:\IFL\grasp_stable` points to `cefd0af`.
That revision mixes grasp code with incomplete food-sorting imports and is not
a coherent version of either product. It also contains generated build state
and untracked recovery files.

The historical pure-grasp release is commit `c3e7a66`. It remains in the local
Git object database through `refs/stash`, although no current local or remote
branch names it. Its previous release record reports 632 passing non-actuating
tests, successful Python checks, and successful isolated ROS package builds.

## Goal

Create a new independent pure-grasp worktree from `c3e7a66`, preserve the
existing sorting workspace without modification, archive useful recovery
documentation, verify the new source tree, and only then remove the current
half-finished worktree.

## Required Runtime Behavior

The recovered workspace is a grasping system, not a food-sorting system.

It must retain the historical interactive experiment-session workflow:

```text
Available objects (18): ...
Enter up to 6 required objects, comma-separated (blank = all random):
```

The behavior is:

- one to six names keep those objects in the requested order and randomly
  fill the remaining scene slots;
- a blank line samples six unique objects from the complete object pool;
- grasp categories select the strategy for each sampled object but do not
  constrain the blank-input scene composition.

The separate `random` scene mode remains category-aware: it selects one object
from each grasp category plus one additional unique object. It must not be
confused with blank `assign` input, which is the user's preferred fully random
scene.

The recovered perception pipeline must have no dependency on
`my_course_pkg.tasks.sorting`, food/non-food classification, or RGB-D bin
localization.

## Worktree Layout

Keep the sorting workspace unchanged:

```text
E:\IFL\PraktikumSoSe26_ML for Robotics
  branch: feature/gui-pipeline
```

Create the pure-grasp workspace as:

```text
E:\IFL\grasp_stable_pure
  branch: codex/pure-grasp
  code baseline: c3e7a66
```

After the new worktree passes static verification and its documentation is
preserved, remove:

```text
E:\IFL\grasp_stable
  branch: grasp_stable
  current commit: cefd0af
```

The `grasp_stable` branch reference may remain for historical comparison. The
worktree removal does not require deleting the branch or changing the remote.

## Preservation Rules

The sorting workspace has local state and is outside the recovery mutation
scope. Do not clean, reset, switch, or rewrite it.

Before removing the half-finished worktree, preserve its useful untracked
documents in the new pure-grasp worktree:

- root `AGENTS.md` becomes the startup instruction for reading the new handoff;
- root `HANDOFF.md` is archived under `docs/recovery/` before being replaced by
  a pure-grasp-specific root handoff;
- root `HOWTO.md` is archived under `docs/recovery/` because parts describe the
  half-finished August code and must not remain the active run guide.

Also restore the historical July release handoff from the Git history into
`docs/agent_handoff.md` for detailed provenance.

Do not carry these generated or incorrect artifacts into the recovered
workspace:

- `build/`, `install/`, and `log/` from the half-finished worktree;
- the untracked partial `tasks/sorting/` restoration;
- Python caches or other generated output.

## Recovery Sequence

1. Resolve and verify the main repository Git directory from Windows.
2. Create `codex/pure-grasp` directly at `c3e7a66`, preserving that otherwise
   unnamed commit with a durable branch reference.
3. Add `E:\IFL\grasp_stable_pure` as a linked worktree for that branch.
4. Verify the new worktree before copying or editing documentation.
5. Archive the current handoff/HOWTO, restore the historical release handoff,
   and create a concise root handoff for the recovered workspace.
6. Commit documentation separately so the parent of the documentation commit
   remains exactly `c3e7a66` and runtime provenance stays explicit.
7. Re-run static verification after documentation changes.
8. Remove `E:\IFL\grasp_stable` with `git worktree remove --force` from the main
   repository. Do not use a raw recursive filesystem deletion.
9. Prune only stale worktree metadata after confirming both retained
   worktrees appear correctly in `git worktree list`.
10. Open `E:\IFL\grasp_stable_pure` in a fresh Dev Container and perform a
    clean build and runtime validation there.

## Static Verification Before Removal

The new worktree must satisfy all of the following before the old worktree is
removed:

- its branch is `codex/pure-grasp`;
- the runtime baseline resolves to `c3e7a66` before the documentation-only
  commit;
- `pipeline.py`, `llm_sam2.py`, and `foundationpose.py` contain no
  `tasks.sorting` imports;
- `tasks/sorting/` is absent;
- `experiment_session.launch.py` exists;
- the prompt includes `blank = all random`;
- blank assigned-object input selects six unique objects from the full pool;
- explicit assigned objects remain first and are filled without duplicates;
- `mix`, category-aware `random`, and `assign` remain separately implemented;
- the sorting worktree's branch, commit, tracked changes, and untracked files
  are unchanged.

## Container Build and Runtime Verification

The recovered worktree must use a fresh build tree. Do not copy generated
artifacts from the removed worktree and do not alternate build modes in one
build directory.

Inside the new Dev Container:

1. install or verify the Python runtime dependencies used by the ROS Python;
2. run focused scene-selection and launch-prompt tests;
3. run the pure-grasp selector/planner/executor regression tests;
4. build the required ROS workspace packages using one consistent build mode;
5. verify imports for the perception pipeline without sorting modules;
6. launch the experiment session and confirm the object prompt appears;
7. use blank object input and confirm six unique full-pool objects are selected;
8. stop before physical grasp motion unless the user explicitly requests the
   live trial after reviewing the generated scene.

## Failure Handling

- If `c3e7a66` cannot be resolved, stop before creating or deleting a worktree.
- If the destination path already exists, inspect it and stop; do not overwrite
  it.
- If static verification fails, retain the current half-finished worktree and
  diagnose the new worktree.
- If documentation preservation fails, retain the current half-finished
  worktree.
- If worktree removal reports a target mismatch, stop and re-run
  `git worktree list`; never substitute a recursive delete.
- If the new container build fails, keep the recovered source worktree and
  repair only its generated build environment.

## Acceptance Criteria

- Two independent local worktrees remain: one sorting version and one pure
  stable-grasp version.
- The sorting worktree is byte-for-byte untouched by the recovery operation.
- The pure-grasp code lineage is explicitly rooted at `c3e7a66`.
- The pure-grasp runtime contains no food-sorting dependency.
- The interactive experiment-session prompt supports named objects and blank
  full-pool random selection.
- Useful recovery documentation is preserved, while partial sorting code and
  stale build artifacts are not copied.
- The half-finished `E:\IFL\grasp_stable` worktree is removed only after the
  recovered worktree passes static verification.

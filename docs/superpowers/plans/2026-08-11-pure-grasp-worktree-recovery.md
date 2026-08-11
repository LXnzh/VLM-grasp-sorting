# Pure-Grasp Worktree Recovery Implementation Plan

## Objective

Preserve the existing sorting worktree, recover the July pure-grasp release
from `c3e7a66` into a new linked worktree, preserve useful documentation, and
remove the current half-finished worktree only after verification succeeds.

## Step 1: Record Preconditions

- Confirm `c3e7a66` resolves to a commit.
- Confirm `codex/pure-grasp` does not already exist.
- Confirm `E:\IFL\grasp_stable_pure` does not already exist.
- Record the sorting worktree HEAD and porcelain status.
- Confirm the current half-finished worktree is exactly
  `E:\IFL\grasp_stable` at the expected branch.

Stop without mutation if any identity or destination check fails.

## Step 2: Create the Recovery Reference and Worktree

From the main Windows repository:

```powershell
git branch codex/pure-grasp c3e7a66
git worktree add E:\IFL\grasp_stable_pure codex/pure-grasp
```

Verify the new worktree reports branch `codex/pure-grasp` and commit
`c3e7a66`.

## Step 3: Verify the Pure-Grasp Source

- Confirm `tasks/sorting/` is absent.
- Confirm pipeline perception modules have no sorting imports.
- Confirm `experiment_session.launch.py` exists.
- Confirm the assigned-object prompt contains `blank = all random`.
- Confirm the scene selector keeps `mix`, category-aware `random`, and
  full-pool `assign` behavior distinct.
- Run the host-compatible scene-selection and launch-prompt tests if their
  dependencies are available; otherwise record that they require the ROS
  container and do not treat that as a source-verification failure.

Do not remove the old worktree unless these source checks pass.

## Step 4: Preserve Documentation

- Copy the current root `HOWTO.md` and `HANDOFF.md` into dated files below
  `docs/recovery/` in the new worktree.
- Restore the historical July `docs/agent_handoff.md` from the release history.
- Add root `AGENTS.md` requiring future agents to read the new handoff.
- Add a new root `HANDOFF.md` describing the pure-grasp branch, fully random
  experiment-session workflow, build status, and remaining container steps.
- Copy the approved recovery design and this implementation plan into the new
  branch.
- Commit only documentation, with `c3e7a66` as the commit's direct parent.

Verify runtime source remains identical to `c3e7a66` after the documentation
commit.

## Step 5: Remove the Half-Finished Worktree

From the main Windows repository:

- resolve the exact old and new absolute paths;
- confirm both appear in `git worktree list`;
- confirm preserved documentation exists in the new worktree;
- run `git worktree remove --force E:\IFL\grasp_stable`;
- run `git worktree prune` only after removal succeeds;
- verify the sorting and pure-grasp worktrees remain registered.

Do not use `Remove-Item -Recurse`, `rmdir`, or another raw recursive deletion.

## Step 6: Final Host Verification

- Confirm the sorting worktree HEAD and porcelain status match the precondition
  snapshot.
- Confirm `codex/pure-grasp` exists and its runtime tree matches `c3e7a66`.
- Confirm `E:\IFL\grasp_stable` no longer exists.
- Confirm `E:\IFL\grasp_stable_pure` contains the archived and active handoff
  documents.

## Step 7: Container Handoff

The user opens `E:\IFL\grasp_stable_pure` in a fresh Dev Container. Then:

- create a fresh MuJoCo virtual environment;
- install ROS-side Python dependencies in the system Python used by `ros2`;
- perform one consistent clean build;
- run focused scene/launch and grasp regression tests;
- start `experiment_session.launch.py`;
- leave the assigned-object line blank and verify six unique full-pool objects
  are generated;
- require explicit user authorization before proceeding into robot motion.

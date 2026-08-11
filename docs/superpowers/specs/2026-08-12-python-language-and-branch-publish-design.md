# Python Language Statistics And Branch Publish Design

## Goal

Publish the current integration work to
`https://github.com/LXnzh/VLM-grasp-sorting` from a branch named
`integrate-stable-tracking-sorting`, without a `codex/` prefix, and make
Python the repository's majority language after the change reaches the default
branch.

## Current State

The local branch is `codex/integrate-stable-tracking-sorting` and is eight
commits ahead of its remote tracking branch. The worktree contains only the
unrelated untracked `.obsidian/` directory, which must remain untracked.

GitHub currently reports:

- Jupyter Notebook: 3,441,718 bytes;
- Python: 1,155,417 bytes;
- C++: 248,462 bytes;
- CMake, Shell, and Dockerfile: 32,751 bytes combined.

All Jupyter bytes come from
`src/ifl_air_mujoco_sim/examples/test_render.ipynb`. The file contains about
3.6 KB of cell source and 3.43 MB of generated execution output.

The desired new branch name is available on the remote. The old remote branch
is associated only with merged PR #1, so deleting it will not close or retarget
an active PR.

## Linguist Change

Add exactly one path-specific rule to `.gitattributes`:

```gitattributes
src/ifl_air_mujoco_sim/examples/test_render.ipynb linguist-generated=true
```

This keeps the Notebook and its outputs unchanged while excluding that
generated-output artifact from GitHub's language statistics. Do not apply a
global `*.ipynb` rule, clear Notebook outputs, delete files, or mark unrelated
assets as generated or vendored.

With the current remote language byte counts and the Notebook excluded, Python
is expected to represent about 80% of the remaining recognized source bytes.
GitHub performs the authoritative asynchronous recalculation; the repository
homepage reflects the new ratio only after the commit is merged into the
default `main` branch.

## Branch And Publish Flow

After the written specification is approved:

1. Add the Linguist rule and verify it with `git check-attr`.
2. Stage only `.gitattributes`; never stage `.obsidian/`.
3. Commit the Linguist change on the current branch.
4. Rename the local branch to `integrate-stable-tracking-sorting`.
5. Push the renamed branch to `origin` with upstream tracking.
6. Verify the new remote branch resolves to the local HEAD.
7. Delete the old remote branch
   `codex/integrate-stable-tracking-sorting`.
8. Verify the old remote branch is absent.
9. Create a draft pull request from `integrate-stable-tracking-sorting` into
   `main`, summarizing the complete integration diff and validation evidence.

Do not force-push or push directly to `main`. If authentication, push, old
branch deletion, or PR creation fails, stop and report the exact completed and
incomplete external state rather than retrying with a broader operation.

## Verification

Before publishing:

- `git check-attr linguist-generated -- <notebook>` returns `true`;
- `git diff --check` passes;
- the staged diff contains only the intended `.gitattributes` rule;
- the previously completed product validation remains recorded: 454
  `my_course_pkg` functional tests passed and the package built successfully.

After publishing:

- the local branch name has no `codex/` prefix;
- `origin/integrate-stable-tracking-sorting` points to local HEAD;
- `origin/codex/integrate-stable-tracking-sorting` no longer exists;
- the draft PR targets `main` from the renamed branch;
- `.obsidian/` remains untracked and unpushed.

## Non-Goals

- Altering Notebook content or outputs.
- Falsifying language statistics by relabeling source files as Python.
- Excluding future notebooks from language statistics.
- Merging the pull request or pushing directly to `main`.
- Changing application, ROS, simulation, perception, planning, or grasp code.

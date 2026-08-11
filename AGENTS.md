# Repository Agent Instructions

- Before diagnosing, building, or changing this repository, read
  [`HANDOFF.md`](HANDOFF.md) completely.
- Consult [`docs/agent_handoff.md`](docs/agent_handoff.md) for the detailed
  historical stable-grasp record when investigating prior behavior.
- This worktree is the pure stable-grasp product. Do not add food/non-food
  sorting, bin localization, PBVS, tracking, or GUI-pipeline dependencies
  unless the user explicitly changes its product scope.
- Treat `HANDOFF.md` as the current record of verified state and already-tried
  operations. Update it after resolving a blocker or materially changing the
  repository/build state.
- Run Git commands from Windows. The Dev Container mounts only this linked
  worktree and cannot resolve its Windows `.git` pointer to the main Git
  directory.
- Run ROS, MuJoCo, Python dependency, build, test, and runtime commands inside
  the Dev Container.
- Do not preserve backward compatibility. Remove obsolete paths instead of
  adding compatibility layers, fallbacks, or migrations.
- Choose the simplest implementation that fully meets current requirements.
- Keep components modular and concerns clearly separated.
- Prefer established dependencies already used by the project before adding
  packages or reimplementing common functionality.
- Preserve unrelated user changes and generated experiment evidence.

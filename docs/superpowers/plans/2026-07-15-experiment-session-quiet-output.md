# Experiment Session Quiet Output Implementation Plan

**Goal:** Keep the integrated session terminal interactive by redirecting all
long-running MuJoCo/MoveIt runtime output to logs without changing standalone
launch defaults or finite pipeline/grasp output.

**Approved design:**
`docs/superpowers/specs/2026-07-15-experiment-session-quiet-output-design.md`

## Working-Tree Constraints

Both host workspaces are dirty and the Docker container mounts the sibling
`-grasp-stable` workspace at `/home/ws`. Preserve all existing changes. Commit
only this plan in the source workspace; implement with narrow patches, then
synchronize only the affected files to the mounted workspace.

## Step 1: Add RED Output-Routing Tests

Modify launch tests to require:

- `experiment_session.launch.py` passes `quiet_runtime_output:=true`;
- the full launch defaults quiet mode to `false` and forwards it to both nested
  launches;
- MuJoCo selects log-only output only in quiet mode and `screen` normally;
- MoveIt and gripper select log-only output only in quiet mode while retaining
  their existing normal output modes; and
- the supervisor-owned interface always writes its log but does not call its
  screen writer when session quiet mode is enabled.

Run the focused tests and confirm the new assertions fail before production
changes.

## Step 2: Implement Session-Scoped Quiet Launch Output

Add a `quiet_runtime_output` launch argument, default `false`, to the full
launch and both nested launch files. Propagate it through the existing includes.
Use a small pure selector so affected actions retain their exact current output
when false and use an explicit stdout/stderr-to-log map when true. Pass `true`
only from `experiment_session.launch.py`.

Do not change standalone launch commands, ROS severity, RViz behavior, or
finite reset/pipeline/grasp streaming.

## Step 3: Make Owned Interface Log-Only In Session Mode

Add an explicit output-mirroring option to `OwnedInterfaceProcess`, preserving
its reusable default. Construct it from `ExperimentSession` with mirroring
disabled. The reader thread must still drain stdout and flush every line to
`trial_NNN/moveit2_iface.log` so quiet mode cannot deadlock or discard logs.

Run the focused tests until green and run focused flake8/compilation checks.

## Step 4: Synchronize, Build, And Regress

Synchronize the affected launch files, supervisor, tests, design, and plan to
the mounted `/home/ws` workspace. Run the core session/launch tests plus shared
scene and grasp regressions. Build `ifl_air_ur_launch` and `my_course_pkg` with
`--symlink-install`, source the result, and inspect installed launch arguments.

## Step 5: Live Non-Motion Smoke Test

Start the integrated launch only far enough to confirm that MuJoCo remains open,
the graph gate passes, and `Instruction:` remains visible without continuous
runtime output. Do not submit an instruction or execute grasp motion. Stop with
Ctrl+C, record log locations and results, and update `docs/agent_handoff.md`.

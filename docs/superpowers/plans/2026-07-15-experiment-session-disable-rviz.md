# Experiment Session Disable RViz Implementation Plan

**Goal:** Start integrated experiment sessions with MuJoCo visible and both RViz
windows disabled, without changing standalone launch defaults.

**Architecture:** Add explicit `false` values for the two existing RViz launch
arguments at the `experiment_session.launch.py` include boundary. Extend the
existing launch-action test so this isolation is executable and regression
protected.

## Task 1: Add the failing launch-argument assertions

- Modify `src/ifl_air_ur_launch/test/test_experiment_session_launch.py`.
- In the existing session-action test, require both `launch_robot_rviz` and
  `launch_moveit_rviz` to equal the literal string `false`.
- Run the focused test and confirm it fails before implementation.

## Task 2: Disable RViz only for experiment sessions

- Modify `src/ifl_air_ur_launch/launch/experiment_session.launch.py`.
- Pass both existing RViz arguments as `false` to the full MuJoCo/MoveIt launch.
- Do not edit defaults in any included launch file.
- Rerun the focused test and the complete `ifl_air_ur_launch` test directory.

## Task 3: Verify and hand off

- Run Python compilation, fatal flake8 checks, and an
  `ifl_air_ur_launch` symlink build in the container.
- Verify the installed launch module contains both explicit false arguments.
- Update `docs/agent_handoff.md` without staging unrelated dirty changes.
- Do not launch a simulator automatically; the result applies on the user's
  next experiment-session restart.

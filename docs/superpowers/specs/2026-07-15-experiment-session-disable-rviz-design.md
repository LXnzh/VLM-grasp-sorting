# Experiment Session: Disable RViz by Default

## Goal

When starting `experiment_session.launch.py`, open the MuJoCo visualization
without opening either RViz visualization. Keep standalone launch behavior and
all simulation, perception, planning, and grasp behavior unchanged.

## Existing Behavior

`experiment_session.launch.py` includes
`cell_small_full_mujoco_moveit.launch.py` but does not pass either RViz launch
argument. The included launch already defaults the robot RViz to `false`, while
the MoveIt RViz defaults to `true`; therefore an RViz window opens for an
experiment session.

## Design

Change only the launch arguments passed by `experiment_session.launch.py`:

- pass `launch_robot_rviz=false`;
- pass `launch_moveit_rviz=false`.

Do not change the defaults in `cell_small_full_mujoco_moveit.launch.py` or its
nested robot/MoveIt launch files. This confines the new behavior to the
integrated experiment-session workflow and preserves RViz for standalone use.
MuJoCo remains launched through the same existing include and is otherwise
untouched.

## Failure and Compatibility Behavior

Both arguments already exist and are forwarded to RViz `IfCondition` gates, so
no new runtime branch or error handling is needed. Other experiment-session
arguments, startup ordering, quiet output, scene selection, and supervisor
behavior remain unchanged.

## Verification

Add or extend launch tests to assert that the experiment-session include passes
both RViz arguments as `false`, while the MuJoCo/full-stack include remains
present. Run the focused `ifl_air_ur_launch` tests, Python compilation, fatal
flake8 checks, and the package build. Do not start a new simulator during
automated verification.

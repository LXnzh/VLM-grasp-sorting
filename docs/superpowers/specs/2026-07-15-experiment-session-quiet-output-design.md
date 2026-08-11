# Experiment Session Quiet Output Design

## Goal

Keep the terminal used by `experiment_session.launch.py` reliably interactive.
While the supervisor is waiting for an instruction or an Enter/`q` decision,
continuous MuJoCo, MoveIt, gripper, robot-state, and owned `moveit2_iface`
messages must not overwrite the prompt or interrupt typing.

## Scope

The quiet behavior applies only to the integrated experiment session. Existing
standalone launch commands retain their current screen output by default.
Pipeline, reset, and `grasp_demo` output may still be streamed while those
finite stages are running because none of them runs concurrently with an input
prompt.

## Design

Add a boolean quiet-runtime launch argument, defaulting to `false`, at the full
cell launch boundary and propagate it into the nested robot/gripper and MoveIt
launch files. The experiment-session launch passes `true` explicitly.

When quiet-runtime mode is enabled:

- MuJoCo and ROS-node actions use an explicit stdout/stderr-to-log destination
  map (Humble's `output="log"` alias still mirrors stderr to the screen);
- `move_group`, the gripper adapter, and robot-state publisher use `log` output
  mode;
- RViz uses the same explicit log-only destination map; and
- the supervisor-owned `moveit2_iface` continues writing its per-trial log but
  does not mirror each line to the interactive terminal.

When quiet-runtime mode is disabled, every affected launch file preserves its
existing output behavior (`screen` or `both`). This avoids changing manual
debugging workflows.

The supervisor remains the only component that writes concurrently with its
input prompts. It continues to display trial boundaries, `Instruction:`,
failure summaries, exact per-stage log paths, and the Enter/`q` prompt.

## Failure Handling and Logs

Quieting means redirecting rather than discarding output. Launch-managed
MuJoCo and ROS-node output remains in the normal ROS launch log directory.
The owned interface remains in `trial_NNN/moveit2_iface.log`. Pipeline,
`grasp_demo`, reset, graph-readiness, and process-probe logs remain in their
existing per-session paths under `/tmp/my_course_experiment_sessions`.

If readiness or a trial stage fails, the supervisor keeps the existing safe
pause behavior and prints the relevant per-stage log path before accepting
Enter or `q`.

## Verification

- Launch tests verify that the integrated session enables quiet-runtime mode.
- Launch-helper tests verify quiet and normal output selection and propagation.
- Supervisor tests verify interface output is written to its log without being
  mirrored to the terminal in session mode.
- Existing session, launch, scene-selection, perception, and grasp regression
  suites remain green.
- Rebuild both affected ROS packages and run a live smoke test confirming that
  `Instruction:` stays usable while MuJoCo remains open.

## Non-Goals

- Do not suppress pipeline or `grasp_demo` stage output while those commands
  are actively executing.
- Do not change ROS logging severity or hide logs from standalone launches.
- Do not treat the repeated missing-knuckle warning as part of this display
  fix; it remains separately diagnosable from the redirected logs.

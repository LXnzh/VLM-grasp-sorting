# Persistent Experiment Session Design

## Goal

Replace the current five-to-six-terminal trial workflow with one persistent ROS
launch session. Startup securely obtains the VLM API key, asks once for up to
six scene objects, starts the MuJoCo/MoveIt stack, and then repeatedly runs a
reset-perception-grasp trial loop. MuJoCo and the selected scene remain alive
between trials.

The integration must preserve the existing safety behavior: a failed stage
does not fall through to later commands, a failed perception run can never use
stale pose output, and motion is allowed only when the ROS graph has exactly
one expected interface/server instance.

## User Interaction

The normal entrypoint is a new top-level launch file:

```bash
ros2 launch ifl_air_ur_launch experiment_session.launch.py
```

Before starting any child process, it performs two startup interactions:

1. If `VLM_API_KEY` is present in the launch environment, reuse it without
   displaying its value. Otherwise prompt once with hidden input.
2. Display the 18 configured object names and accept one comma-separated line
   containing at most six names. Reuse the existing `assign` parsing rules:
   input is case-insensitive, whitespace around names is ignored, duplicates,
   unknown names, empty tokens, and more than six names retry the whole line.
   A blank line means six random objects.

The selected identities and placement slots are fixed for the entire session.
Every `/reset_sim` restores that same initial scene; it does not prompt again
or reroll the objects.

After a successful trial, the session displays:

```text
Press Enter for the next trial, or q to quit:
```

Enter starts the next safe trial boundary. `q` shuts down the interface, the
base launch, MuJoCo, and the session. While this prompt is idle, MuJoCo and the
currently owned interface remain running.

## Architecture

Use a persistent ROS launch plus a dedicated Python trial supervisor.

`experiment_session.launch.py` owns the complete session lifetime. It resolves
the secret and scene selection before child startup, includes the existing
full simulation stack, starts the supervisor, and shuts down the complete
launch when the supervisor exits.

The top-level launch owns both startup prompts in one blocking setup action.
Extract/reuse the current YAML loading and `assign` parsing helpers instead of
copying the 18-name list. After validation, pass the normalized object list to
the included full launch explicitly. Add an optional non-interactive
`assigned_object_names` launch argument for this handoff: direct
`scene_mode:=assign` use still prompts when the argument is absent, while the
session wrapper supplies it and therefore cannot prompt twice. The supervisor
is returned/started only after the key and object list have both been resolved.

The existing launch hierarchy is adjusted only to make ownership explicit:

- `moveit_cell_small_ur_orbbec_robotiq.launch.py` conditionally includes
  `arm_api2/moveit2_iface.launch.py` behind a new `launch_moveit_iface`
  argument.
- `cell_small_full_mujoco_moveit.launch.py` forwards the argument and keeps its
  default `true`, preserving existing launch behavior.
- `experiment_session.launch.py` includes the full launch with
  `scene_mode:=assign` and `launch_moveit_iface:=false`.
- The trial supervisor is then the only owner of the standalone
  `moveit2_iface` process for this session.

The base launch continues to own MuJoCo, robot/gripper support, `move_group`,
RViz, and the remaining long-lived nodes. The supervisor must not duplicate or
restart those components between trials.

## API-Key Lifetime

The launch checks the inherited environment before MuJoCo starts. If no usable
`VLM_API_KEY` exists, use hidden terminal input rather than a visible ROS launch
argument. An empty value retries or fails clearly before children start.

The resolved key is kept only in the launch/session process environment and is
inherited by pipeline subprocesses. It must never be:

- written to a repository file or generated configuration;
- included in a command-line argument;
- printed in launch, supervisor, exception, or test output; or
- persisted after the complete session exits.

The existing real-looking key in `my_course_pkg/howto.md` must be removed and
replaced with safe `export VLM_API_KEY=...` guidance. Because a committed or
shared plaintext key must be treated as compromised, the user must rotate it;
the integration cannot make the old value safe retroactively.

## Process Ownership And Interface Restart

The supervisor starts this existing interface launch as its own subprocess
group:

```bash
ros2 launch arm_api2 moveit2_iface.launch.py \
  robot_name:=ur \
  launch_joy:=false \
  launch_servo_watchdog:=false
```

It retains the exact process/group handle. On a subsequent trial it sends
`SIGINT` to that owned group, waits for graceful exit, and escalates only
within that same owned group if a bounded shutdown timeout is exceeded. It
must not use a global `pkill` to terminate processes it did not create.

After the owned process exits, run the equivalent of
`pgrep -a -x moveit2_iface`. The required result is empty. A remaining process
is foreign or stale, so the trial fails closed and pauses for inspection; the
supervisor reports it but does not silently kill it.

After starting the replacement interface, the supervisor waits for readiness
and verifies ROS graph uniqueness before reset or motion. The gate requires:

- exactly one `/moveit2_iface` node;
- exactly one `/arm/move_to_pose` action server;
- exactly one `/arm/state/current_pose` publisher;
- exactly one gripper command action server; and
- no duplicated exact node names among the session's safety-critical nodes.

Timeout, missing endpoints, or duplicate endpoints is a failed stage. Client
counts are not used as uniqueness gates.

## Trial State Machine

The first trial and every retry use the same safe boundary. On initial startup
the supervisor has no old owned interface to stop; otherwise it first performs
the owned shutdown and empty-process verification.

The ordered stages are:

1. Stop the previously owned interface when present.
2. Verify that no `moveit2_iface` executable remains.
3. Start one owned interface and pass the readiness/uniqueness gate.
4. Call `/reset_sim` and require a successful `std_srvs/srv/Trigger` response.
5. Wait for the existing bounded post-reset settling period.
6. Prompt for a new natural-language instruction.
7. Run `ros2 run my_course_pkg pipeline`, supplying that instruction to its
   stdin while leaving its normal stdout/stderr visible.
8. Only if pipeline exits with code zero, run
   `ros2 run my_course_pkg grasp_demo` automatically.
9. Only if grasp exits with code zero, display the Enter/`q` trial menu.

Reset is intentionally performed for the first trial as well as later trials,
so there is one auditable boundary and no special first-run state. A new
pipeline is mandatory after every reset. The supervisor must not invoke grasp
when pipeline fails, times out, is interrupted, or produces an unsuccessful
exit status.

The supervisor should reuse the command-running abstractions already present
in `grasp_eval.py` where practical, but session control, user input, and owned
process lifecycle should remain in a focused module rather than expanding the
evaluation CLI into a second mode.

## Failure And Recovery

Any stage failure stops the state machine at that point. The supervisor prints
an explicit diagnostic containing:

- trial number and failed stage;
- the sanitized command or ROS operation;
- exit code, signal, timeout, or failed readiness condition; and
- the complete visible child stdout/stderr needed for inspection.

Each session creates a timestamped log directory outside the source tree. Each
trial and stage writes to a separate file while streaming the same output to
the terminal. The failure report prints the relevant absolute log path. Log
files include commands, stage transitions, and child output, but never the API
key or a complete environment dump.

Diagnostics must redact the API key and must not dump the complete child
environment. No automatic retry, reset, perception fallback, grasp, or motion
occurs after failure.

The failure prompt waits indefinitely:

```text
Trial stopped. Inspect/fix the problem, then press Enter to restart from the
safe interface boundary, or q to quit:
```

Enter restarts at interface shutdown/verification rather than resuming the
failed command. This makes recovery consistently perform interface restart,
simulation reset, and fresh perception. `q`, Ctrl+C, closed stdin, and parent
launch shutdown all enter the same cleanup path.

## Shutdown Behavior

On normal quit or interruption, the supervisor first sends `SIGINT` to its
owned interface group and waits for it. Its exit triggers shutdown of the
top-level launch, which terminates the long-lived base stack and MuJoCo. Launch
shutdown must also terminate the supervisor so neither side can leave the
other running.

Cleanup is idempotent: repeated shutdown signals do not start a new trial,
issue a reset, or launch another interface. The process returns a nonzero code
for unrecovered stage failure or cleanup failure and zero for an explicit
normal `q` exit.

## Compatibility And Scope

Existing users of
`cell_small_full_mujoco_moveit.launch.py` retain its embedded interface by
default. Existing `mix`, `random`, and `assign` selection semantics and direct
simulator entrypoints remain unchanged. The new session wrapper intentionally
uses `assign` directly and therefore does not show the older scene-mode prompt.

This feature does not change grasp planning/execution, perception models,
object aliases, object placement coordinates, reset semantics, collision
clearance, controller settings, or the success criteria of `pipeline` and
`grasp_demo`.

## Test Plan

Add unit tests using fake input, command runners, process groups, clocks, and
ROS graph probes for:

- startup reuse of an exported API key and one-time hidden prompt fallback;
- rejection of an empty prompted key and absence of the key from logs/errors;
- direct 18-object display and existing assign input behavior;
- explicit normalized assigned-name forwarding bypassing the inner prompt,
  while direct `scene_mode:=assign` without names still prompts once;
- exact first-trial and subsequent-trial stage order;
- pipeline success automatically starting grasp;
- pipeline failure, timeout, and interruption always skipping grasp;
- grasp failure entering the failure pause;
- Enter retry restarting at the full safe boundary and `q` shutting down;
- owned-group `SIGINT`, bounded wait, and idempotent cleanup;
- empty `pgrep` acceptance and foreign/stale-process fail-closed behavior;
- readiness success, missing endpoints, timeouts, and duplicate ROS graph
  endpoints; and
- sanitization of commands and environment in failure reports.
- terminal streaming plus per-stage log creation and failure-path reporting.

Add launch-focused tests that prove:

- the existing full launch defaults `launch_moveit_iface` to `true`;
- the new wrapper forwards `false` and owns exactly one standalone interface;
- all scene selection finishes before children start;
- supervisor exit requests complete launch shutdown; and
- existing full-launch arguments and `mix`/`random`/`assign` tests regress
  unchanged.

Run focused unit/launch tests with project plugin autoload disabled where
required, Python compilation, and symlink builds for `my_course_pkg` and
`ifl_air_ur_launch`. Automated verification must not launch MuJoCo or execute
robot motion. The first live validation is user-run and staged: startup/scene
selection, API inheritance, graph uniqueness, interface restart, reset, and
failure recovery are checked before allowing a real grasp trial.

## Documentation And Workspace Synchronization

Update `my_course_pkg/howto.md` to describe the single session command, secure
API export, Enter/`q` loop, stage-failure recovery, and legacy full-launch
compatibility. Remove the plaintext key and retire the manual multi-terminal
sequence for this workflow.

The active Docker container may mount the sibling `-grasp-stable` workspace.
Before synchronizing implementation, compare each affected file, preserve
unrelated dirty changes in both workspaces, and copy only the approved feature
delta. Do not automatically start the simulator or robot while implementing or
testing this design.

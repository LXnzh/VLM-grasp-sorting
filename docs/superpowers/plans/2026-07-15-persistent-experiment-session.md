# Persistent Experiment Session Implementation Plan

**Goal:** Add one persistent ROS launch session that securely resolves the VLM
API key, prompts once for up to six scene objects, keeps MuJoCo alive, and
repeatedly performs an exclusively owned interface restart, simulation reset,
fresh instruction/pipeline, and automatic `grasp_demo`. Every failure pauses
with inspectable logs and never falls through to motion.

**Approved design:**
`docs/superpowers/specs/2026-07-15-persistent-experiment-session-design.md`

## Working-Tree Constraints

The workspace is already dirty. In particular,
`cell_small_full_mujoco_moveit.launch.py`, the simulator assign-mode files, and
the untracked `test_scene_mode_launch.py` contain approved scene-selection work
that must be preserved. Before each edit, inspect the current diff and apply
only the session delta. Stage and commit only explicitly named files.

The active Docker container mounts the sibling `-grasp-stable` workspace at
`/home/ws`. Implement and test in the current host workspace first, then
compare affected files before synchronizing only this feature to the mounted
workspace. Do not start MuJoCo or execute robot motion during automated work.

## Step 1: Write Launch-Startup Tests First

Modify:

- `src/ifl_air_ur_launch/test/test_scene_mode_launch.py`

Add:

- `src/ifl_air_ur_launch/test/test_experiment_session_launch.py`

Extend the existing scene-mode tests with RED coverage for a new
`assigned_object_names` launch argument:

- the default sentinel means direct `scene_mode:=assign` still prompts once;
- an explicit normalized list bypasses the inner prompt;
- an explicit empty list means six random objects and also bypasses the prompt;
- explicit names are parsed through the existing canonical-name validator;
- invalid/duplicate/over-six explicit names fail before child startup; and
- the resulting Hydra override remains exactly one validated list.

The new top-level launch tests must cover pure startup helpers and action
construction without launching ROS:

- an inherited, non-empty `VLM_API_KEY` is reused without calling `getpass`;
- a missing key invokes hidden input once, while blank input retries;
- the key is absent from printed messages, action descriptions, commands, and
  exceptions;
- `ROS_DOMAIN_ID`, `VLM_CANDIDATE_OVERRIDE`, and
  `FOUNDATIONPOSE_MASK_INDEX` are removed from the child environment, making
  the new instruction authoritative while retaining `VLM_API_KEY`;
- all 18 names come from the existing YAML helper and the assign prompt occurs
  before any returned child action;
- blank object input is serialized as an explicit empty assignment, not as the
  inner-prompt sentinel;
- the included full launch receives `scene_mode:=assign`, the normalized names,
  and `launch_moveit_iface:=false`;
- the legacy full/MoveIt launch path retains its current watchdog default
  `true`, while the supervisor's separately owned interface command explicitly
  passes `launch_servo_watchdog:=false`;
- the supervisor command contains no secret and starts only after startup
  resolution; and
- supervisor exit emits a complete launch `Shutdown` event.

Load launch modules by file path, following the existing test pattern. Use
fake contexts and injected input/getpass/output functions so no real terminal,
ROS graph, or secret is needed.

Run the focused tests and require the new assertions to fail before
implementation:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  src/ifl_air_ur_launch/test/test_scene_mode_launch.py \
  src/ifl_air_ur_launch/test/test_experiment_session_launch.py -q
```

## Step 2: Add Explicit Launch Ownership And Startup Resolution

Modify:

- `src/ifl_air_ur_launch/launch/cell_small_full_mujoco_moveit.launch.py`
- `src/ifl_air_ur_launch/launch/moveit_cell_small_ur_orbbec_robotiq.launch.py`
- `src/ifl_air_ur_launch/package.xml`

Add:

- `src/ifl_air_ur_launch/launch/experiment_session.launch.py`

In the existing full launch:

- declare `launch_moveit_iface` with default `true` and forward it to the
  MoveIt child launch;
- declare/forward `launch_servo_watchdog` with default `true`, matching the
  current embedded `arm_api2` behavior;
- declare `assigned_object_names` with an unambiguous interactive sentinel;
- when `scene_mode` is `assign`, prompt only for the sentinel; otherwise parse
  and validate the explicit value through the existing assign helpers; and
- serialize explicit zero through six names into the same Hydra override used
  by the current implementation.

In the MoveIt child launch, declare the same two arguments, put the existing
`arm_api2` include behind
`IfCondition(LaunchConfiguration("launch_moveit_iface"))`, and forward both
`launch_joy` and `launch_servo_watchdog` when the interface is enabled.
Preserve the default embedded-interface and enabled-watchdog behavior for every
existing full-launch caller.

Implement the top-level launch as one early `OpaqueFunction`. It must:

1. resolve the API key from the launch context environment or hidden input;
2. load and prompt for the object names using the current YAML/assign helpers;
3. remove stale ROS-domain and manual perception-selection overrides from the
   child environment;
4. return the full-launch include with explicit assigned names and embedded
   interface disabled;
5. return one `ExecuteProcess` for
   `ros2 run my_course_pkg experiment_session`; and
6. register `OnProcessExit` for that exact supervisor action to emit
   `Shutdown` for the full session.

Do not use a launch argument, command argument, `SetEnvironmentVariable`
logging action, or generated file for the API key. Update only the in-memory
launch context environment inherited by child processes, and never print its
value.

Add explicit runtime dependencies for `my_course_pkg` and `arm_api2` to the
launch package. Run the Step 1 tests until GREEN, then compile all three launch
files.

## Step 3: Write Supervisor State-Machine Tests First

Add:

- `src/my_course_pkg/test/test_experiment_session.py`

Design the production module around injected interfaces for user input,
clock/sleep, command execution, owned-interface lifecycle, ROS graph probing,
and logging. First add RED tests for:

- exact first-trial order:
  verify empty process -> start interface -> readiness -> reset -> settle ->
  prompt instruction -> pipeline -> grasp -> success menu;
- exact later-trial order, including stop-owned-interface before empty-process
  verification;
- blank instructions retrying locally without running pipeline;
- pipeline input being exactly the new instruction plus one newline;
- pipeline success automatically starting grasp with no confirmation prompt;
- reset failure, readiness failure, pipeline failure/timeout/interruption, and
  grasp failure stopping immediately and skipping all later stages;
- the failure prompt waiting until Enter or `q`;
- Enter after any failure restarting from the complete interface boundary,
  never from the failed command;
- Enter after success starting the same boundary and `q` performing one
  idempotent cleanup before exit;
- Ctrl+C/EOF using the same cleanup path; and
- explicit normal quit returning zero while unrecovered failure/cleanup error
  returns nonzero.

Represent stage results with a small immutable dataclass containing stage,
return code, timeout/interruption state, sanitized message, and log path. Keep
the orchestration state machine independent of ROS imports so its tests run on
the host and in the system ROS Python environment.

## Step 4: Implement The Trial Supervisor Core

Add:

- `src/my_course_pkg/my_course_pkg/experiment_session.py`

Modify:

- `src/my_course_pkg/setup.py`
- `src/my_course_pkg/package.xml`

Add the console script:

```text
experiment_session = my_course_pkg.experiment_session:main
```

Implement constants for the existing commands:

```text
ros2 service call /reset_sim std_srvs/srv/Trigger "{}"
ros2 run my_course_pkg pipeline
ros2 run my_course_pkg grasp_demo
ros2 launch arm_api2 moveit2_iface.launch.py robot_name:=ur
  launch_joy:=false launch_servo_watchdog:=false
```

Implement `ExperimentSession` as the tested dependency-injected state machine.
The first trial has no owned interface to stop; every later trial and every
retry stops the exact owned process first. Require reset command exit code zero
and a parsed `Trigger` response with `success=True`; a transport-level zero exit
with `success=False` is still failure. Use the existing three-second post-reset
settle behavior before prompting for the instruction.

Pipeline inherits the session environment, including `VLM_API_KEY`, but the
supervisor constructs a sanitized child environment without stale
`VLM_CANDIDATE_OVERRIDE`, `FOUNDATIONPOSE_MASK_INDEX`, or `ROS_DOMAIN_ID`.
Preserve deliberate grasp configuration/debug variables rather than silently
rewriting them. Invoke grasp only after a zero pipeline exit.

Add declared runtime dependencies for the ROS CLI commands and `std_srvs` as
appropriate. Run the state-machine tests until GREEN.

## Step 5: Write Process, Graph-Gate, And Logging Tests First

Extend `test_experiment_session.py` with RED tests for the concrete adapters:

- interface launch uses `subprocess.Popen(..., start_new_session=True)` and
  records the exact handle/process group;
- shutdown sends `SIGINT`, then bounded `SIGTERM`/`SIGKILL` only to that owned
  group when necessary;
- repeated cleanup is a no-op and never starts another process;
- no production command contains `pkill`;
- `pgrep -a -x moveit2_iface` accepts empty output and reports non-empty
  output as a foreign/stale-process failure without killing it;
- node-list parsing requires one `/moveit2_iface`, one
  `/robotiq_2f_urcap_adapter`, one `/robot_state_publisher`, and one
  `/move_group`; optional `/servo_watchdog_node` may be absent but never
  duplicated;
- action parsing requires one `/arm/move_to_pose` server and one
  `/robotiq_2f_urcap_adapter/gripper_command` server;
- topic parsing requires one `/arm/state/current_pose` publisher;
- missing endpoints retry until the readiness deadline, while any duplicate
  endpoint fails closed immediately;
- synchronous pipeline/grasp commands stream output and return the correct
  code/timeout/interruption result;
- every session creates a timestamped directory under
  `/tmp/my_course_experiment_sessions` with separate trial/stage logs;
- terminal output and log output contain the same child lines; and
- failure diagnostics include the absolute stage-log path but redact the API
  key and never serialize the full environment.

Use fake `Popen` objects, fake kill/wait functions, canned ROS CLI output, and
temporary directories. No test may inspect or signal a real process.

## Step 6: Implement Owned Processes, Readiness Gates, And Tee Logging

In `experiment_session.py`, implement:

- an owned interface-process class that starts a new POSIX session/process
  group, pumps its combined stdout/stderr to screen and its current interface
  log, and stops only that group;
- a synchronous tee command runner that writes pipeline/grasp/reset output to
  both terminal and stage logs while enforcing timeouts;
- a process probe for the exact `pgrep` command;
- pure parsers for `ros2 node list`, `ros2 action info`, and
  `ros2 topic info --verbose` output;
- a bounded readiness loop that distinguishes temporarily missing endpoints
  from immediately unsafe duplicates; and
- a log manager that sanitizes command metadata and prints the relevant
  absolute path on failure.

Use a reader thread plus queue (or an equivalently bounded nonblocking design)
so a child that stops producing newline-terminated output cannot defeat the
timeout. Join output threads during cleanup. Signal handlers set a shutdown
flag and route control through the idempotent cleanup path rather than starting
new subprocesses from inside the handler.

Run all supervisor tests until GREEN, then compile the module.

## Step 7: Update Documentation And Remove The Exposed Secret

Modify:

- `src/my_course_pkg/my_course_pkg/howto.md`
- `docs/agent_handoff.md`

Replace the manual multi-terminal instructions with:

```bash
cd /home/ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export VLM_API_KEY='...'
ros2 launch ifl_air_ur_launch experiment_session.launch.py
```

Document the hidden-prompt fallback, one-time 18-object selection, persistent
MuJoCo behavior, Enter/`q` loop, failure log location, and safe retry boundary.
Retain a short compatibility note that the old full launch still owns its
embedded interface by default and must not be combined with a standalone one.

Delete the plaintext key from the current file without quoting it in terminal
output, tests, assistant messages, or commit messages. Record that it must be
revoked/rotated because repository removal does not invalidate an exposed
credential or erase it from existing Git history.

Update the handoff with implementation state, exact verification results,
remaining live validation, and the new command only after tests/builds are
complete.

## Step 8: Focused Regression, Build, And Installed-Artifact Verification

In the Docker-mounted workspace, after carefully synchronizing only affected
files, run:

```bash
cd /home/ws
source /opt/ros/humble/setup.bash
source install/setup.bash
unset ROS_DOMAIN_ID

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  src/ifl_air_ur_launch/test/test_scene_mode_launch.py \
  src/ifl_air_ur_launch/test/test_experiment_session_launch.py \
  src/my_course_pkg/test/test_experiment_session.py \
  src/my_course_pkg/test/test_grasp_eval.py -q

/usr/bin/python3 -m py_compile \
  src/ifl_air_ur_launch/launch/cell_small_full_mujoco_moveit.launch.py \
  src/ifl_air_ur_launch/launch/moveit_cell_small_ur_orbbec_robotiq.launch.py \
  src/ifl_air_ur_launch/launch/experiment_session.launch.py \
  src/my_course_pkg/my_course_pkg/experiment_session.py

colcon build --packages-select my_course_pkg ifl_air_ur_launch \
  --symlink-install

source install/setup.bash
ros2 launch ifl_air_ur_launch experiment_session.launch.py --show-args
ros2 pkg executables my_course_pkg | grep experiment_session
```

Also run the existing simulator/assign and grasp-selector regressions because
the full launch and installed package metadata are shared:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH=/home/ws/src/ifl_air_mujoco_sim:/home/ws/src/ifl_air_mujoco_sim/.venv/lib/python3.10/site-packages:$PYTHONPATH \
/usr/bin/python3 -m pytest \
  src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py \
  src/ifl_air_ur_launch/test/test_scene_mode_launch.py -q

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  src/my_course_pkg/test/test_target_aliases.py \
  src/my_course_pkg/test/test_foundationpose_mask_matching.py \
  src/my_course_pkg/test/test_grasp_selector.py -q
```

`--show-args`, executable discovery, tests, compilation, and builds are
non-motion checks. Do not run the normal session launch during automated
verification because it would start MuJoCo and expose a motion-capable stack.

## Step 9: User-Run Live Validation

Hand the new command to the user for staged live validation. Before any real
grasp, require these observations in order:

1. exported-key reuse and hidden-prompt fallback never display the key;
2. exactly 18 object names appear and the selected identities remain unchanged
   after reset;
3. the graph gate reports exactly one interface, arm action server, pose
   publisher, gripper server, MoveIt node, gripper adapter, and robot-state
   publisher;
4. Enter after a deliberately harmless failed instruction pauses and restarts
   from interface shutdown/empty verification/reset;
5. the logged `pgrep` result is empty between owned interface instances;
6. one fresh pipeline success automatically starts one `grasp_demo`; and
7. `q` removes the owned interface and shuts down MuJoCo/the entire launch.

Only after gates 1-5 pass should the user allow a real motion trial. Never run
that live or motion validation automatically from Codex.

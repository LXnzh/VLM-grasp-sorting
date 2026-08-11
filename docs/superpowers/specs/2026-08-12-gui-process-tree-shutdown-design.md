# GUI-Owned Process Tree Shutdown Design

## Problem

`gui_manager.py` starts the simulation stack and grasp tasks with
`subprocess.Popen`, but closing the Tk window only destroys the GUI. ROS launch,
MoveIt, action servers, the simulator, and task processes can remain alive.
Starting a new GUI and scene then creates duplicate nodes and duplicate action
servers, which produces mismatched goal/result responses and unsafe-looking
robot motion.

## Required behavior

The GUI owns every process that it starts. Closing the main window or leaving
`mainloop()` must terminate all GUI-owned process trees, including:

- active grasp and PBVS tasks;
- the food-sorting ROS launch process;
- MuJoCo, MoveIt, arm action servers, watchdogs, gripper adapters, RViz, and
  every other descendant of that launch;
- short-lived GUI commands that are still running.

The next GUI must also refuse to start a food-sorting stack while an existing
full-stack launch process is alive outside its own process registry. This
defensive check prevents a second stack after an abnormal exit that could not
run cleanup.

## Process ownership and shutdown

Each `Popen` command starts a new Linux session with `start_new_session=True`.
Its PID is therefore also the process-group ID for the complete command tree.
The existing `active_processes` registry remains the single source of ownership.

Shutdown is idempotent:

1. Mark the GUI as closing so no new tasks can start and repeated close events
   do nothing.
2. Snapshot every still-running registered process.
3. Send `SIGTERM` to each owned process group.
4. Wait for all groups using one bounded shutdown deadline.
5. Send `SIGKILL` to groups that remain alive and reap their root processes.
6. Clear the registry and destroy the Tk window.

The Tk `WM_DELETE_WINDOW` callback invokes shutdown. `main()` also invokes the
same cleanup in a `finally` block, covering terminal interruption and ordinary
`mainloop()` exit. `SIGKILL`, container failure, and host failure cannot execute
cleanup; the startup guard covers their possible leftovers.

## Existing-stack guard

Before `start_food_scene()` launches anything, it checks the container process
table for the exact full-stack launch command
`cell_small_full_mujoco_moveit.launch.py`. A matching process already registered
by this GUI is handled by the existing "already running" path. Any other match
is treated as an orphan or externally owned stack: the GUI refuses to launch a
second stack and displays a clear cleanup message.

The guard does not globally kill processes by name. Automatic global `pkill`
could terminate a deliberately hand-started ROS session. The current known
orphan stacks are cleaned once during implementation and verified separately.

## Error handling

An already-exited process group is harmless during cleanup. Other termination
errors are collected and shown in the terminal rather than preventing cleanup
of the remaining groups. The GUI still closes after the bounded cleanup pass.
No compatibility path or additional dependency is introduced.

## Tests and acceptance

Automated tests verify that:

- `Popen` creates a new session;
- normal close sends `SIGTERM` to every live owned process group and reaps it;
- a process that exceeds the deadline receives `SIGKILL`;
- repeated cleanup is harmless;
- an existing external full-stack launch blocks `start_food_scene()`;
- the existing same-GUI duplicate-start guard remains intact.

Runtime acceptance requires one-time termination of both currently running
stacks and the active grasp task, followed by ROS graph checks confirming that
duplicate MoveIt, arm action server, robot-state publisher, watchdog, and
gripper nodes are gone. A fresh GUI must then start exactly one scene, and
closing it must leave no full-stack launch or grasp process alive.

## Non-goals

This change does not modify PBVS, tracking, grasp planning, scene generation,
or ROS action behavior. It only fixes GUI process ownership and lifecycle.

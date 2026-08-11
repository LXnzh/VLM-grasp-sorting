# Experiment Session DDS Graph Settle Design

## Goal

Prevent a safe owned-interface restart from being rejected solely because ROS
2 discovery temporarily retains the previous `moveit2_iface` endpoints. Keep
the existing fail-closed behavior when duplicate endpoints persist or the
required graph never becomes complete and unique.

## Observed Failure

Trial 2's owned interface received SIGINT and exited, and Trial 3's exact
pre-start `pgrep -a -x moveit2_iface` check was empty. After the replacement
interface registered, the ROS action graph briefly reported two same-name arm
servers even though the operating system contained only the new process. A
later graph query reported one server, showing that the second endpoint was a
stale DDS discovery record.

`RosGraphProbe.wait_until_ready` currently retries missing endpoints until the
normal readiness timeout but returns a failure on the first duplicate sample.
That makes a transient discovery overlap indistinguishable from a persistent
unsafe duplicate.

## Design

Keep `evaluate_graph_snapshot` strict and unchanged: every snapshot containing
more than one required node or endpoint remains classified as duplicate.
Change only the temporal policy in `RosGraphProbe`.

On the first duplicate snapshot, start an independent 30-second duplicate
settle window. Continue polling and write every query and evaluation to the
existing `graph_ready.log`.

- If duplicate state disappears before the settle deadline, continue normal
  readiness evaluation.
- A ready snapshot does not immediately pass. Require two consecutive ready,
  unique snapshots, separated by the existing polling interval.
- A missing, failed-query, or duplicate snapshot resets the consecutive-ready
  count.
- If duplicate state is still present when its 30-second settle deadline is
  reached, fail immediately with the latest duplicate details.
- The existing overall interface-readiness timeout remains authoritative for
  missing endpoints and query failures. It must not be extended by the
  duplicate settle window.

Expose the duplicate settle duration and required clean-sample count as
validated `SessionConfig` values with defaults of 30 seconds and 2 samples.
Pass them into the default `RosGraphProbe`. They do not need new launch/CLI
arguments because this is an internal safety policy, not an experiment control.

## Logging and User Feedback

The graph log records when duplicate settling begins, subsequent duplicate
observations, clean-sample progress such as `1/2`, and the final pass or
persistent-duplicate failure. The interactive terminal remains quiet while the
gate polls. On failure, the existing supervisor message and exact
`graph_ready.log` path remain unchanged.

## Safety Boundaries

- Do not kill processes based on graph counts. The existing owned-process
  teardown and exact `pgrep` boundary remain separate and unchanged.
- Do not restart the ROS daemon or alter the ROS domain.
- Do not accept a single clean sample after a duplicate.
- Do not turn a duplicate into an unconditional wait for the full 180-second
  interface timeout.
- A duplicate that persists for 30 seconds stops the trial before reset,
  pipeline, FoundationPose, or robot motion.

## Verification

Add deterministic unit tests with injected monotonic time, sleep, and ROS CLI
results for:

- transient duplicate followed by two clean snapshots passes;
- one clean snapshot followed by another duplicate resets the clean streak;
- duplicate persisting through 30 seconds fails with duplicate details;
- missing endpoints still use the existing overall timeout;
- an initially clean graph requires two consecutive samples;
- configured CLI query timeouts remain applied to every graph command.

Run the focused experiment-session tests, package regression tests, Python
compilation, and a Docker `colcon build --symlink-install` for
`my_course_pkg`. A live validation should restart one completed trial and show
duplicate-settle messages only in `graph_ready.log`, followed by either a
two-sample pass or a bounded persistent-duplicate stop.

## Non-Goals

- Identifying DDS endpoint GUIDs or changing middleware lease settings.
- Hiding a genuinely persistent second interface.
- Changing interface ownership, MuJoCo lifetime, scene selection, reset,
  pipeline, grasp execution, or prompt behavior.

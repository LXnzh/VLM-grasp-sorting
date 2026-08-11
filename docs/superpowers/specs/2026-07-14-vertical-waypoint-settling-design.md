# Vertical Waypoint Settling Design

## Goal

Stop treating one lateral feedback sample immediately after a vertical descent
command as a final failure. Give XY the same bounded convergence time already
given to Z, without adding new thresholds, sample windows, configuration, or
object-specific behavior.

The existing 1.5 mm world-XY gate, 3 mm Z gate, 5 mm maximum waypoint spacing,
1.5-second settle timeout, frozen per-run command offset, and fail-before-next-
waypoint rule remain unchanged.

## Live Evidence

The safe-pregrasp stability fix worked in the latest foam-brick run. Calibration
waited through a moving window, then froze after median and newest feedback
passed. Descent waypoint 1 passed.

Waypoint 2 was commanded at `1784059997.4586` and failed at
`1784059997.5022`, about 44 ms later. Its first observed error was 2.1 mm XY
and 2.6 mm Z. Z was already within tolerance, but the descent loop immediately
failed because XY exceeded 1.5 mm by 0.6 mm. The code gives Z up to 1.5 seconds
to converge but gives XY no settling time.

## Selected Minimal Change

Keep the existing pre-command lateral gate. Before sending a lower waypoint,
if the current pose is already more than 1.5 mm laterally from the next nominal
waypoint, hold and fail without lowering.

After a waypoint command, change only the decision order:

1. Read the current raw pose.
2. Compare it with the unmodified nominal waypoint.
3. Succeed only when **both** XY error is at most 1.5 mm and Z error is at most
   3 mm in the same sample.
4. If either error is still outside its gate and the existing 1.5-second
   deadline has not expired, wait the existing 0.05-second command period and
   sample again. Send no additional command.
5. On timeout, hold the latest observed full 6D pose and abort before the next
   waypoint or gripper close.

This removes the immediate post-command XY failure branch. It does not accept
2.1 mm as a completed waypoint; it only allows that sample to settle. No next
5 mm descent command is issued until a later sample simultaneously passes the
original XY and Z gates.

## Rejected Alternatives

- A three-sample stationary window plus a separate emergency bound is more
  machinery than this observed transient requires.
- Counting consecutive XY violations depends on sampling rate and introduces a
  second acceptance rule.
- Widening the XY tolerance would change the physical grasp margin and is not
  necessary.
- Ignoring XY during all descent would permit the next lower command while the
  arm is laterally outside the gate and is not acceptable.

## Scope

### `executor.py`

- In `_execute_vertical_approach_waypoint()`, replace immediate post-command
  lateral failure with a joint XY-and-Z convergence condition.
- Keep the pre-command XY check, single command, timeout, hold behavior,
  nominal-target comparison, frozen offset, and diagnostics.
- Update timeout wording to report waypoint convergence rather than Z-only
  convergence.

### Tests

- Reproduce a first post-command 2.1 mm XY transient followed by an in-gate
  sample; the waypoint passes and only one descent command is sent.
- Prove a persistent lateral error waits and then times out, holding the newest
  pose.
- Prove Z-only error still waits and times out.
- Prove success requires XY and Z to pass in the same sample.
- Preserve the pre-command lateral failure test.
- Preserve nominal-plus-offset commanding and nominal feedback gating.
- Run focused executor tests, related grasp regression, Python compilation, and
  the Docker-mounted `my_course_pkg` symlink build.

## Live Validation

Do not start robot motion automatically. After rebuild and simulator restart,
the user can repeat the bounded foam-brick run with
`GRASP_DEBUG_STOP_AFTER_CLOSE=1`.

The expected log is: waypoint 2 may first report 2.1 mm XY as still settling,
then either passes on a later in-gate sample and continues downward, or times
out and holds if the lateral error is persistent.

## Acceptance Criteria

- A single post-command lateral transient no longer aborts immediately.
- XY and Z retain their exact 1.5 mm and 3 mm gates.
- Both gates must pass together before the next waypoint.
- A waypoint is commanded once; settling never adds commands.
- Persistent error still fails within 1.5 seconds and holds the latest pose.
- No planner, selector, configuration, gripper, return, or non-vertical logic
  changes.

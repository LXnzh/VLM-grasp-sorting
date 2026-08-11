# Vertical Waypoint Pre-command Settling Design

## Goal

Prevent one transient lateral feedback sample between two otherwise successful
vertical descent waypoints from aborting the grasp. Keep the existing physical
limits and never send a lower waypoint while the TCP is laterally outside the
allowed corridor.

The existing 1.5 mm world-XY gate, 3 mm post-command Z gate, 5 mm maximum
waypoint spacing, 1.5-second settling timeout, 0.05-second sampling period,
frozen per-run command offset, and hold-on-timeout behavior remain unchanged.

## Live Evidence

The latest foam-brick run verified that the generic bounds-floor clamp loaded
and correctly preserved a higher valid library target. Safe-pregrasp
calibration converged and descent waypoints 1 through 6 passed.

Waypoint 6 was accepted at approximately 1.5 mm XY and 1.8 mm Z error. About
74 ms later, before waypoint 7 was sent, feedback moved another 1.4 mm in Y.
The waypoint-7 pre-command sample therefore reported 2.9 mm XY and 2.6 mm Z
error. The unchanged immediate pre-command gate held and aborted with zero
waypoint-7 commands. At Z=1.0069 m, the TCP was still about 67.6 mm above the
final target, so this was controller feedback rebound rather than brick or
table contact.

## Considered Approaches

1. **Bounded pre-command lateral settling (selected).** When XY is outside the
   existing gate, wait without sending the next lower command. Continue only
   if feedback returns inside the same gate before the existing timeout.
2. Require multiple consecutive samples before declaring the preceding
   waypoint complete. This adds a new acceptance rule and still cannot
   guarantee that the following sample will not rebound.
3. Remove or widen the pre-command XY gate. This would permit downward motion
   while lateral alignment is outside the validated physical margin and is
   rejected.

## Selected Behavior

For every vertical descent waypoint:

1. Read the current raw TCP feedback and compare world XY with the unmodified
   nominal next waypoint.
2. If XY is within 1.5 mm, send the existing single compensated waypoint
   command immediately.
3. If XY is outside 1.5 mm, start a bounded wait using the existing 1.5-second
   timeout and 0.05-second sampling period. Send no waypoint or compensation
   command while waiting.
4. If a later sample returns inside 1.5 mm, send the lower waypoint once and
   begin the existing post-command joint XY/Z convergence loop with its own
   full timeout.
5. If XY remains outside the gate until the deadline, hold the latest observed
   6D pose and raise `VerticalApproachConvergenceError` before any lower
   waypoint or gripper command.

Only XY participates in this pre-command wait. The robot is expected to be
approximately one 5 mm step above the next waypoint, so checking next-waypoint
Z at this stage would incorrectly reject normal staged descent. After the
command, the existing rule still requires XY at most 1.5 mm and Z at most 3 mm
in the same sample.

Diagnostics log when pre-command waiting begins, when lateral alignment
recovers, and the latest pose/error on timeout. They do not add commands or
object-specific branches.

## Scope

### `executor.py`

- Replace the immediate pre-command lateral failure with the bounded XY-only
  wait described above.
- Reuse `FINAL_APPROACH_SETTLE_TIMEOUT_SEC` and
  `FINAL_APPROACH_COMMAND_PERIOD_SEC`; add no configuration.
- Preserve nominal feedback gating, compensated command generation, the
  post-command convergence loop, orientation, and hold behavior.

### Tests

- Reproduce the recorded pre-command 2.9 mm transient followed by an in-gate
  sample; prove exactly one lower waypoint is sent afterward.
- Prove persistent pre-command lateral error times out, holds the newest pose,
  and never sends the lower waypoint.
- Preserve post-command transient recovery, persistent error timeout,
  same-sample XY/Z acceptance, command-offset, and waypoint-spacing tests.
- Run focused executor tests, related grasp regression, Python compilation,
  and the Docker-mounted `my_course_pkg` symlink build.

## Acceptance Criteria

- A transient pre-command lateral rebound can settle for at most 1.5 seconds.
- No lower command is sent while XY exceeds 1.5 mm.
- Returning inside the original XY gate sends exactly one lower command.
- Persistent drift holds the latest pose and aborts before further descent.
- The 1.5 mm XY and 3 mm Z tolerances are unchanged.
- No planner, selector, configuration, gripper, return, non-vertical, or
  object-specific behavior changes.

## Live Validation

Do not start robot motion automatically. After rebuilding and restarting the
simulator, the user can repeat a bounded foam-brick close-only run. The expected
log either reports pre-command lateral recovery followed by waypoint 7, or a
timeout/hold with no waypoint-7 command if the drift is persistent.

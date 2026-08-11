# Global Vertical Z Tolerance Design

## Goal

Allow the current `gelatin_box` vertical grasp to continue when a descent
waypoint settles 5.2 mm above its nominal Z target. Make the smallest possible
runtime change by increasing the existing global vertical Z tolerance from
3 mm to 6 mm.

## Runtime Evidence

The latest task completed VLM selection, tracking, FoundationPose, planning,
pregrasp calibration, and the first 18 of 20 vertical descent waypoints. At
`waypoint_19_of_20`:

- XY error was 0.3 mm, inside the existing 2 mm descent tolerance;
- signed Z error was +5.2 mm;
- the existing 3 mm Z tolerance rejected the waypoint;
- the visible open gripper already straddled the `gelatin_box`.

The user explicitly selected a global 6 mm tolerance and accepted that the
task will proceed to the final descent waypoint rather than close immediately
at waypoint 19.

## Selected Change

Change only the default returned by
`read_vertical_approach_z_tolerance_m()`:

```text
GRASP_VERTICAL_APPROACH_Z_TOLERANCE_M: 0.003 -> 0.006
```

Keep the existing environment-variable interface and validation. Do not add a
new setting, object-specific condition, execution result, or control-flow
branch.

The existing constant is intentionally global for the vertical grasp profile.
The new 6 mm symmetric tolerance therefore applies to:

- vertical pregrasp convergence;
- every intermediate vertical descent waypoint;
- the ordinary final vertical waypoint gate;
- final vertical close-readiness verification.

Both positive and negative Z residuals up to and including 6 mm are accepted.
Residuals greater than 6 mm still fail through the existing hold-and-raise
path.

## Preserved Behavior

Do not change:

- the 1.5 mm pregrasp XY tolerance;
- the 2 mm descent and final XY tolerance;
- the 5 mm maximum vertical waypoint spacing;
- the 1.5-second settle timeout and 50 ms sample period;
- pregrasp command-offset calibration and its limits;
- live vertical bounds or controlled-center final acceptance;
- waypoint commands, including the final downward waypoint;
- gripper opening, closing, effort, lift, placement, or return behavior;
- non-vertical grasp profiles;
- VLM, tracking, PBVS, FoundationPose, planning, GUI, or simulator behavior.

For the observed run, waypoint 19 may pass at +5.2 mm, after which waypoint 20
is still commanded. Closing occurs only after the remaining waypoint and the
existing final verification pass.

## Verification

Update deterministic configuration and executor tests to prove:

- the default vertical Z tolerance is 0.006 m;
- the existing environment override still works and retains its validation;
- positive and negative 5.2 mm vertical residuals pass the ordinary gate;
- absolute residuals greater than 6 mm fail and hold the latest observed pose;
- XY tolerances, waypoint spacing, timeout, and command count are unchanged;
- non-vertical execution behavior is unchanged;
- waypoint 19 success continues to waypoint 20 rather than closing early.

Run the focused configuration and executor tests, the complete functional
`my_course_pkg` test suite, Python compilation, fatal Flake8 on changed Python
files, `git diff --check`, and a container build of `my_course_pkg`.

Do not launch robot motion automatically. After rebuilding and restarting the
GUI-owned stack, the user may repeat a `gelatin_box` trial. The expected log is
that a +5.2 mm waypoint residual passes under the 6 mm ordinary gate, the final
waypoint is commanded, and the task either closes after final verification or
still fails safely if a later residual exceeds the unchanged 6 mm limit.

## Non-Goals

- Closing early at waypoint 19.
- Adding bounds-aware intermediate-waypoint acceptance.
- Changing grasp geometry or target height.
- Diagnosing or compensating for the source of the Z tracking residual.
- Relaxing any XY condition or any non-vertical grasp policy.

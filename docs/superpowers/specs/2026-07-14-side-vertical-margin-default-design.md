# Side-Grasp Vertical-Margin Default Design

**Date:** 2026-07-14
**Status:** Approved in conversation; additional review explicitly waived

## Goal

Allow normal lift-return trials to run with:

```bash
ros2 run my_course_pkg grasp_demo
```

without requiring a repeated shell override for the validated side-grasp
vertical safety envelope.

## Decision

Change the default value of `GRASP_SIDE_APPROACH_VERTICAL_MARGIN_M` from
`0.05 m` to `0.12 m` for every object that uses the `side` grasp profile.

This is a physical gripper-envelope setting, not a tomato-specific tuning
value. The same gripper has approximately `0.104 m` TCP-to-lowest-finger reach,
so the `0.12 m` default covers that reach with margin and matches the existing
round-top vertical-envelope default.

The environment variable remains supported, so an explicit experiment can
still override the default. No shell profile or wrapper script is added.

## Scope

- Change only the side-profile default in `config.py`.
- Add a regression test for the `0.12 m` default and retained environment
  override behavior.
- Keep the corridor radius and horizontal clearance margin unchanged.
- Keep round-top, vertical, centered, and top-down profile behavior unchanged.
- Keep `lift_return`, release clearance, grasp selection, and execution logic
  unchanged.

## Verification

- Configuration tests prove the unset default is `0.12 m` and an explicit
  environment value is still honored.
- Existing side-corridor geometry tests continue to pass.
- Related planner, selector, executor, and plan-only regression passes.
- `my_course_pkg` rebuilds successfully.
- An installed-code check reports `SIDE_GRASP_APPROACH_VERTICAL_MARGIN_M=0.12`.

## Alternatives Not Selected

- A shell-profile export is machine-specific and would not follow the code
  through team merge.
- A wrapper script creates another entrypoint that can be bypassed accidentally.
- A tomato-only special case would incorrectly model a shared physical gripper
  dimension as an object-specific parameter.

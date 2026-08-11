# Racquetball Alias Expansion Implementation Plan

## Objective

Extend the existing deterministic racquetball perception alias so ordinary
instructions containing `racquetball` or `blue ball` receive the same strict
blue-ball grounding behavior as `blue racquetball`.

## Step 1: Expand the Alias Matcher

- Update `perception/target_aliases.py` so the existing alias recognizes the
  three approved singular, space-separated phrases case-insensitively.
- Keep the canonical target, SAM2 prompt, accepted class name, visual hint,
  override precedence, and strict failure behavior unchanged.
- Continue rejecting plural, underscore-joined, and lookalike phrases covered
  by the existing negative tests.

## Step 2: Update Focused Regression Tests

- Add `racquetball` and `blue ball` instructions to positive alias tests.
- Exercise the pipeline wrapper for both newly supported instructions and
  require `blue ball.` as the SAM2 text prompt.
- Retain tests proving invalid near-matches, explicit overrides, verifier
  uncertainty, and stale-pose invalidation remain safe.

## Step 3: Verify Without Motion

Inside the Dev Container:

- run target-alias and FoundationPose mask-matching tests;
- run the related perception regression tests;
- compile changed Python files and run fatal flake8 checks;
- rebuild `my_course_pkg` with `--symlink-install` and verify the installed
  alias resolver.

No ROS graph, simulator motion, or grasp command is part of this plan.

## Step 4: Record the Result

- Update `HANDOFF.md` with the implemented matching forms and verification
  results.
- Review and commit only the racquetball implementation, tests, plan, and
  handoff addition, preserving unrelated user changes.

# Pear Level Bilateral Grasp Design

## Goal

Stabilize only the exact normalized object name `pear` by making the two
Robotiq fingers close at the same world height and about the pear's horizontal
geometry center. Preserve every non-pear selector, planner, executor, gripper,
and trajectory default.

## Evidence

Session `20260716_183037_957297` confirmed that the existing pear short-axis
filter fixed the closing direction but did not make the closing axis level.
Trial 1 stalled at gripper position `0.038 rad`, then lifted with the pear held
at its upper shoulder and visibly wobbling. Trial 3 stalled immediately at
`0.000 rad` and stopped before lift. The image shows the left finger above the
right finger.

Replay of all `7,466` raw pear grasps and the existing eight-way yaw expansion
finds only the current two direction- and opening-safe candidates. Both have a
large closing-axis tilt; adding even a 10-degree levelness filter leaves zero
candidates. Therefore a filter alone cannot solve the failure.

## Isolation Boundary

```text
normalized object name == "pear" -> pear seed gate + level/center synthesis
all other object names           -> existing path byte-for-byte unchanged
```

No shared round-top threshold, gripper effort, close target, approach speed,
lift height, convergence tolerance, or other-object route changes.

## Seed Contract

The existing pear-only direction and width gates remain the seed authority.
They must first retain the known raw-index-5675 wrist pair and continue to
require:

- at most 5 degrees short-axis closing error;
- at least 5 mm margin inside the verified 85.16 mm opening;
- all existing orientation, center, normalized-height, table, and scene gates.

The seed supplies contact depth and wrist sign. It does not supply final roll
or pitch because those components caused the unequal finger height.

## Level Orientation Synthesis

For each safe seed:

1. Derive the pear's shorter stored horizontal bounding-box axis in the object
   frame.
2. Transform that axis into the world frame, remove its world-Z component, and
   normalize it.
3. Use the seed closing-axis sign to choose one of the two 180-degree wrist
   alternatives.
4. Set TCP-X to that signed, horizontal short axis.
5. Set TCP-Z to exact world down.
6. Compute TCP-Y from the right-handed cross product.

The resulting two wrist alternatives are level and differ by 180 degrees.
Synthesis fails closed if any projected axis or rotation is non-finite or
degenerate.

## Center And Depth Synthesis

Measure the seed pear geometry center in seed TCP coordinates. Preserve its
TCP-Z coordinate, but set its final TCP-X and TCP-Y coordinates to zero under
the leveled orientation. This retains the library contact depth while placing
the pear center between the fingers.

The new leveled/centered pose is paired with an exact-name
`GRASP_PEAR_Z_OFFSET=+0.005 m`, 5 mm below the prior pear-only value. Live
inspection showed that the level fingers form more stable bilateral contact a
few millimeters below the upper shoulder. This remains 25 mm above the original
unreachable shared `-0.020 m` target. The offset is applied later in world Z;
it is not folded into or duplicated by synthesis.

## Post-Synthesis Hard Gates

Every synthesized candidate must be rechecked and fail closed unless:

- theoretical finger-height difference across 85.16 mm is at most 2 mm;
- short-axis error is at most the existing 5 degrees;
- opening margin is at least the existing 5 mm;
- seed-to-final orientation correction is at most 20 degrees;
- geometry-center XY offset passes the existing 10 mm gate;
- normalized height passes the existing round-top interval;
- world-down approach and table clearance pass existing gates.

The two selected candidates are ranked with the existing round-top score and
then pass through existing scene-clearance and MoveIt reachability checks.

## Pear Close-Readiness Gate

The generic close result currently accepts any stalled position above
`0.010 rad`. That admitted the invalid `0.038 rad` single-shoulder contact.

For pear only, derive a candidate-specific minimum useful close position from
the verified effective opening and the synthesized projected width:

```text
expected_contact_position =
    GRIPPER_CLOSED_POSITION
    * (effective_opening - projected_width)
    / effective_opening

minimum_pear_close_position =
    expected_contact_position - 0.010 rad calibration tolerance
```

The current leveled geometry predicts about `0.17 rad`, so both recorded
failures remain far below the lower bound. A pear close must first pass the
existing generic result checks and then report a finite actual position at or
above the candidate-specific minimum. Otherwise execution holds and raises
before any lift. No retry, stronger effort, target replay, or zero-motion stall
acceptance is added.

## Diagnostics

Pear logs report, per candidate:

- seed short-axis error, width, and opening margin;
- seed-to-final orientation correction;
- final closing-axis world-Z component and finger-height difference;
- final center offset, normalized height, width, and margin;
- expected and minimum acceptable close positions;
- actual close position and the pear close-gate decision.

Non-pear log formats remain unchanged.

## Staged Live Validation

1. Stop at final grasp before close. Require visibly level fingers on opposite
   long sides of the pear and the new diagnostics to pass.
2. Stop after close. Require actual close position to pass the pear-specific
   minimum and require bilateral contact without ejection.
3. Set the existing lift height to 30 mm for a pear-only trial and stop after
   lift. Require the pear to remain centered with no visible rocking.
4. Clear debug overrides and complete three fresh-scene 200 mm lift-and-hold
   trials before declaring pear stable.

The 30 mm stage is a validation override, not a changed shared default.

## Automated Verification

- Pure synthesis produces finite right-handed rotations with TCP-Z world down.
- Both wrist signs are retained and differ by 180 degrees.
- Finger-height difference is at most 2 mm.
- Final center offset, short-axis error, width margin, correction, height, and
  table gates pass.
- The real library produces exactly the known two synthesized alternatives.
- A pear close at `0.000` or `0.038 rad` fails before lift.
- A pear close above the derived minimum proceeds.
- Every configured non-pear object bypasses synthesis and the pear close gate.
- Focused selector, planner, trajectory, executor, and non-target regressions
  pass, followed by compile, fatal flake8, symlink build, and installed-runtime
  checks.

## Acceptance

- Final pear fingers are level within the 2 mm theoretical bound and centered
  about the pear geometry center.
- Invalid early stalls cannot proceed to lift.
- Pear completes three consecutive fresh-scene stable lifts without visible
  rocking.
- No non-pear result or shared default changes.

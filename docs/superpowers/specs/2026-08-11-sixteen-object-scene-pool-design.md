# Sixteen-Object Scene Pool Design

## Goal

Remove `tuna_fish_can` and `pudding_box` from scene-generation choices so
every supported scene mode uses the same 16-object pool. The interactive
experiment prompt must report 16 available objects, and neither excluded
object may be selected explicitly or sampled randomly.

## Scope

This change affects scene configuration and scene-selection regression tests.
It applies to `mix`, category-aware `random`, and interactive or explicit
`assign` modes, including the blank `assign` input that samples six unique
objects.

The change does not remove or alter downstream grasp strategies, YCB name
mappings, FoundationPose support, grasp-library data, or local YCB model
assets. Those components remain available as historical implementation and
runtime data but are no longer reachable through scene generation.

## Source of Truth

`src/ifl_air_mujoco_sim/env/config/base_env.yaml` remains the sole source of
truth for the scene object pool. The implementation will remove the complete
`objects` entries for `tuna_fish_can` and `pudding_box`, rather than add a
second exclusion list or an `enabled` flag.

The same configuration's `scene_object_categories` will remove
`tuna_fish_can` from `cylindrical_can` and `pudding_box` from `box`. This keeps
the category-aware selector consistent with the reduced object pool. The
remaining categories are still valid because `tomato_soup_can` remains in
`cylindrical_can`, and `gelatin_box`, `sponge`, `foam_brick`, and
`rubiks_cube` remain in `box`.

## Resulting Object Pool

The ordered pool contains exactly these 16 objects:

1. `tomato_soup_can`
2. `gelatin_box`
3. `banana`
4. `apple`
5. `lemon`
6. `peach`
7. `pear`
8. `orange`
9. `plum`
10. `sponge`
11. `hammer`
12. `baseball`
13. `tennis_ball`
14. `racquetball`
15. `foam_brick`
16. `rubiks_cube`

The existing order of all retained objects remains unchanged.

## Runtime Behavior

The launch helper already derives the interactive choices directly from the
YAML `objects` list. Consequently, reducing that list changes the prompt to
`Available objects (16)` and makes explicit attempts to assign either removed
name fail through the existing unknown-object validation.

The simulator already passes the same `objects` list to every selector. The
blank assigned-object workflow and partial assignment fill therefore sample
only from the 16 retained objects. `mix` random fill uses the same reduced
pool, while `random` uses the updated categories plus a sixth unique object
from the reduced pool. No new runtime branch or compatibility path is needed.

## Validation

Regression coverage will establish that:

- the runtime YAML contains exactly 16 uniquely named object entries;
- neither `tuna_fish_can` nor `pudding_box` occurs in the scene object pool or
  scene categories;
- the interactive loader returns the same 16 retained names in YAML order;
- explicit assignment rejects both removed names;
- blank and partial `assign` sampling never returns either removed name;
- category-aware `random` remains valid with the reduced category members;
- existing `mix`, placement, uniqueness, and six-object-count behavior stays
  intact.

The focused launch and scene-selection tests will run inside the Dev
Container, as required by the repository instructions. Static searches will
also confirm that the two names no longer appear as selectable members in
active scene-generation configuration or positive-selection fixtures. They
may remain in negative tests that prove excluded names are rejected.

## Documentation

After implementation and verification, `HANDOFF.md` will record that the
interactive and simulator scene pool now contains the 16 release-qualified
objects and can no longer generate `tuna_fish_can` or `pudding_box`. Existing
historical design and recovery records remain unchanged.

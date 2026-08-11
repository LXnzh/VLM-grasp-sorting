# Sixth Random Slot Clearance Design

## Goal

Make the sixth, randomly selected object visible to the fixed perception camera
and graspable without weakening approach collision checks or changing any
object-specific grasp behavior.

## Failure Evidence

In the observed lemon trial, perception and round-top grasp selection produced
eight valid candidates. Before any robot motion, the exact-bounds approach
filter rejected all eight because the candidate TCP paths were only
`0.0841-0.0878 m` from the hammer box, below the required `0.1100 m` corridor
clearance.

A live `/scene_clearance_bounds` sample placed the hammer box near
`(-0.538, 0.246)` with XY size about `0.183 x 0.333 m`, and the lemon center
near `(-0.361, 0.203)`. The current random slot is therefore genuinely too
close to the hammer for the configured gripper envelope; this is not the old
circumscribed-circle false rejection.

The first slot-only correction moved slot 6 from `[-0.36, 0.20, 0.0]` to
`[-0.25, 0.20, 0.0]`. It cleared the hammer, but a fresh live run exposed a
second constraint: the lemon center at approximately `(-0.251, 0.203)` in
`base_link` projected to pixel `(539, 867)`, below the `1280 x 720` image.
The actual lemon was absent from the captured RGB frame. Grounding/SAM instead
returned the banana and apple as two lemon candidates; the verifier reported
low confidence, the score fallback selected the banana, and FoundationPose
therefore placed the alleged lemon inside the live banana bounds. The exact
approach filter correctly rejected all eight resulting candidates at zero
box distance.

## Approved Change

Supersede the first correction and change only placement slot 6 in
`src/ifl_air_mujoco_sim/env/config/base_env.yaml`:

```text
current: [-0.25, 0.20, 0.0]
new:     [-0.85, 0.35, 0.0]
```

The new point is the far, empty camera-visible corner of the existing allowed
table-placement range. Using the live camera transform and current intrinsic
matrix, the lemon geometry center projects to approximately pixel `(742, 110)`.
Using the same live exact bounds, its estimated XY distance to each fixed box
is:

| Fixed object | Distance | Margin beyond `0.110 m` |
| --- | ---: | ---: |
| `foam_brick` | `0.1430 m` | `0.0330 m` |
| `hammer` | `0.2216 m` | `0.1116 m` |
| `tomato_soup_can` | `0.4071 m` | `0.2971 m` |
| `banana` | `0.4432 m` | `0.3332 m` |
| `apple` | `0.6021 m` | `0.4921 m` |

The nearest fixed obstacle is therefore the foam brick, with `33 mm` of
clearance beyond the unchanged approach requirement before fresh-pose
variation. The position remains on the table and inside the existing configured
`x=[-0.85,-0.25]`, `y=[-0.15,0.35]` placement limits.

Update only the corresponding expected `placement_slots` value in
`src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py`.

## Preserved Behavior

- Keep placement slots 1-5 unchanged, including the validated hammer slot 5
  coordinate `[-0.55, 0.30, 0.0]`.
- Keep the five fixed representative objects and their order unchanged.
- Keep `random_object_count: 6`, so exactly one non-representative object fills
  slot 6.
- Keep all object orientations unchanged.
- Keep the round-top corridor radius, safety margin, exact-bounds algorithm,
  grasp selector, planner, executor, and object-specific grasp settings
  unchanged.

## Validation

1. Run `test_populate_scene_selection.py` and require all scene-selection tests
   to pass with the new slot-6 coordinate.
2. Confirm the active YAML and its test expectation contain identical six-slot
   lists and that only slot 6 changed.
3. Rebuild/relaunch the simulator before live validation because placement
   configuration is consumed during scene construction.
4. Confirm `/scene_clearance_bounds` places lemon near the new slot, then reset
   the simulation and rerun fresh lemon perception.
5. Inspect the generated candidate overlay and require the selected mask to
   enclose the visible lemon rather than banana, apple, or another fixed object.
6. Run `grasp_demo` and require at least one lemon `round_top` candidate to pass
   the exact-bounds approach filter without disabling or reducing any clearance
   setting.
7. A successful live result must proceed beyond the former all-candidates-
   rejected failure; final grasp/lift behavior is then evaluated through the
   existing unmodified workflow.

## Non-Goals

- Do not move hammer or any other fixed object.
- Do not modify grasp, motion, or collision-clearance code.
- Do not modify camera pose, field of view, image resolution, or perception
  selection logic.
- Do not add a lemon-specific branch.
- Do not change random-object selection policy or scene object count.
- Do not tune unrelated hammer grasp/lift work in this change.

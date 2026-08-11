# Flat Fixed Initial Scene Design

> Superseded by `2026-07-06-fixed-slots-random-objects-design.md`. The current
> direction keeps object positions fixed but allows four of the six object
> identities to remain random.

## Goal

Create a deterministic initial MuJoCo scene for the current pick-and-place work.
The scene should always spawn exactly these six YCB objects:

- `tomato_soup_can`
- `banana`
- `sponge`
- `hammer`
- `tennis_ball`
- `baseball`

The objects should start flat on the table, separated enough to reduce grasping
and perception interference, while staying within the existing reachable table
area.

## Scope

This design covers only the `flat_fixed` initial scene behavior.

The teammate's stacked-object work is out of scope for this change. A future
`stacked_fixed` scene mode can reuse the same object pool and pick-place loop,
but it should be added as a separate selectable layout rather than replacing
the flat layout.

This change does not introduce a `scene_mode` field yet. For this slice,
`flat_fixed` is represented by the default `base_env.yaml` settings:
non-empty `fixed_object_names`, `random_object_count: 0`, and fixed per-object
`position` values.

## Current Context

The simulation currently defines an 18-object YCB pool in
`src/ifl_air_mujoco_sim/env/config/base_env.yaml`.

Startup selection is controlled by:

- `fixed_object_names`
- `random_object_count`
- `objects`

The current fixed list has five objects and `random_object_count` is also `5`,
so only those fixed objects are selected. Random object fill is effectively not
used, but the configuration still reads as a random-capable scene.

## Proposed Layout

Use a compact 3-column by 2-row grid. This keeps objects away from one another
without spreading them across the full table.

For YCB mesh objects, the configured `position` is not the final MuJoCo body
origin. In the existing mesh-placement code:

- `x` and `y` are the desired world position of the rotated mesh centroid.
- `z` is an additional offset above the table placement calculation.
- `z: 0.0` means the mesh bottom should sit on `table_surface_z` after the
  code applies `body_z = table_surface_z - z_min + z`.

The body pose written into the generated MuJoCo XML is therefore adjusted by
the mesh centroid in XY and by the mesh bottom in Z.

| Object | Position `[x, y, z]` |
| --- | --- |
| `tomato_soup_can` | `[-0.74, -0.08, 0.0]` |
| `sponge` | `[-0.55, -0.08, 0.0]` |
| `hammer` | `[-0.36, -0.08, 0.0]` |
| `banana` | `[-0.74, 0.20, 0.0]` |
| `tennis_ball` | `[-0.55, 0.20, 0.0]` |
| `baseball` | `[-0.36, 0.20, 0.0]` |

These coordinates remain inside the existing table placement range:

- x: `[-0.85, -0.25]`
- y: `[-0.15, 0.35]`

Nearest same-row centroid spacing is about 19 cm. Same-column centroid spacing
is about 28 cm. These values do not prove the object meshes or collision geoms
cannot overlap, especially for asymmetric objects such as `banana` and
`hammer`; the final layout must still be verified in simulation.

The selected six objects intentionally replace the previous five-object
grasp-profile smoke set. All six are supported by the existing YCB object pool
and grasp-name map. Their current grasp profiles remain unchanged:

- `tomato_soup_can`: `side`
- `sponge`: `vertical`
- `banana`, `hammer`: `centered`
- `tennis_ball`, `baseball`: default `top_down`

Because `tennis_ball` and `baseball` use the default profile, and because the
old fixed set included `tuna_fish_can` and `foam_brick`, this layout requires
fresh grasp evaluation rather than assuming the previous success rate carries
over.

## Configuration Changes

Update `base_env.yaml` so:

- `fixed_object_names` contains the six selected objects.
- `random_object_count` is `0`.
- The selected objects use fixed `position` values instead of
  `position_range`.
- The other 12 YCB objects remain in the object pool for future modes, but are
  not selected by the flat fixed scene.

Use this `fixed_object_names` order for deterministic logs and readable diffs:

```yaml
fixed_object_names:
  - "tomato_soup_can"
  - "sponge"
  - "hammer"
  - "banana"
  - "tennis_ball"
  - "baseball"
```

Keep `fixed_object_names` non-empty. With the current
`select_scene_objects()` behavior, `random_object_count: 0` and an empty fixed
list would select the full 18-object pool.

## Data Flow

1. MuJoCo loads `base_env.yaml`.
2. `populate_scene.select_scene_objects()` sees `random_object_count: 0`.
3. Because `fixed_object_names` is non-empty, it selects exactly the fixed
   objects.
4. Each selected mesh object is inserted at its configured fixed position,
   with mesh placement logic correcting XY by mesh centroid and Z by mesh
   bottom so the object sits on the table surface.

## Stacked Mode Compatibility

The future stacked-object work should be added as a separate layout mode, for
example:

- `flat_fixed`: six objects flat on the table.
- `stacked_fixed`: selected objects start stacked and are picked one at a time.
- `random`: optional legacy/debug behavior.

This slice does not add the mode selector. It only makes the default scene the
flat fixed layout.

The stacked mode should not change the flat fixed coordinates. It can share the
same object names, perception pipeline, grasp selector, and executor, but it
will need separate scene placement rules and likely a top-object-first target
selection policy.

## Verification

After implementation:

- Inspect `base_env.yaml` to confirm only the six fixed objects are selected.
- Add or update a scene-selection test confirming `random_object_count=0` with
  six fixed object names returns exactly those six objects in order.
- Run the scene population tests.
- Start the simulator and confirm `/scene_description` contains exactly the six
  fixed objects.
- Visually inspect the table to confirm objects are not overlapping and are
  reachable.
- Check the spawned object extents or collision geoms for overlap, because the
  layout spacing is based on mesh centroids rather than final AABBs.
- Run a fresh grasp evaluation for the six-object flat fixed scene.

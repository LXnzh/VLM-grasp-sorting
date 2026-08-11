# Fixed Slots Random Objects Design

## Goal

Improve grasp repeatability by fixing the initial table positions while keeping
some object variety.

The default scene should:

- always include `tomato_soup_can` and `banana`;
- randomly fill four additional objects from the existing YCB pool;
- spawn the six selected objects at six fixed table slots;
- leave downstream drop/place behavior unchanged.

## Scope

This change only affects initial MuJoCo scene generation.

It does not change:

- grasp selection;
- perception;
- motion execution;
- drop/place target selection;
- teammate stacked-object work.

The previous flat-fixed six-object design is superseded by this design.

## Current Context

`base_env.yaml` currently uses:

- `fixed_object_names: ["tomato_soup_can", "banana"]`
- `random_object_count: 6`
- per-object `position_range`

`select_scene_objects()` already picks fixed objects first and fills the
remaining slots randomly. `populate_scene()` then lays the selected objects out
using randomized grid positions when `position_range` is present.

That means object identity is partly fixed, but object position is still random.

## Proposed Approach

Add an optional top-level YAML field:

```yaml
placement_slots:
  - [-0.74, -0.08, 0.0]
  - [-0.55, -0.08, 0.0]
  - [-0.36, -0.08, 0.0]
  - [-0.74, 0.20, 0.0]
  - [-0.55, 0.20, 0.0]
  - [-0.36, 0.20, 0.0]
```

The selected objects are assigned to these slots in selected-object order.
Because `select_scene_objects()` keeps fixed objects first, the first two slots
are always:

- slot 1: `tomato_soup_can`
- slot 2: `banana`

The four random objects fill slots 3-6 in the order returned by the random
selection.

## Slot Semantics

Each slot has the same meaning as a mesh object's `position` field:

- `x` and `y` are the desired world position of the rotated mesh centroid;
- `z` is an additional offset above the table placement calculation;
- `z: 0.0` means the mesh bottom should sit on `table_surface_z`.

When slots are used, each selected object should be copied and given a fixed
`position`, while its `position_range` is removed. This lets the existing
`populate_scene()` fixed-position branch place the object without changing mesh
placement math.

## Data Flow

1. Hydra loads `base_env.yaml`.
2. `ros2_main.py` and `main.py` pass `placement_slots` into
   `MuJoCoInterface`.
3. `MuJoCoInterface` selects objects using existing `select_scene_objects()`.
4. If `placement_slots` is non-empty, it assigns selected objects to slots.
5. `populate_scene()` receives selected objects with fixed `position` values.
6. Drop/place execution remains unchanged.

## Error Handling

If `placement_slots` is provided, it must contain at least as many slots as the
number of selected objects. Otherwise, scene creation should raise a clear
`ValueError`.

Extra slots are ignored.

## Verification

Add tests to confirm:

- fixed-priority random selection still keeps `tomato_soup_can` and `banana`
  first;
- placement slot assignment removes `position_range` and sets fixed
  `position` values;
- insufficient slots raise a clear error;
- default `base_env.yaml` contains two fixed objects, `random_object_count: 6`,
  and six placement slots.

Run:

```powershell
$env:PYTHONPATH='src/ifl_air_mujoco_sim'; pytest src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py -v
```

Manual simulator verification should confirm the initial object positions are
stable across runs while the four non-fixed object identities can vary.

# Fixed Grasp-Category Representatives Design

## Goal

Make every simulated scene contain one fixed representative of each configured
grasp-selection category so category-specific grasp work can be repeated against
a stable object set.

## Scene Composition

Keep the scene size at six objects. Select these five fixed objects first, in
this order:

1. `tomato_soup_can` for `cylindrical_can`
2. `banana` for `banana`
3. `apple` for `round_top`
4. `foam_brick` for `box`
5. `hammer` for `tool_top`

Fill the sixth position by randomly sampling one object from the remaining
pool. The random object must not duplicate a fixed representative.

Because placement slots are assigned in selection order, the five fixed
representatives occupy the first five existing slots. The sixth, random object
occupies the last slot. No placement coordinates change.

## Implementation

Extend `fixed_object_names` in `base_env.yaml`. Keep
`random_object_count: 6`; this value is the target total scene size, not the
number of randomly selected objects. With five fixed names, the existing
`target_count = max(random_object_count, len(selected))` behavior therefore
adds exactly one random object.

Reuse the existing configuration-driven selection and placement functions
without adding category logic or hard-coded object names to Python runtime
code.

The active implementation workspace is
`E:\IFL\ros2-docker-workspace-vscode-plmrs-grasp-stable`. Apply and verify the
same configuration and test changes there first, then synchronize those
changes back to `E:\IFL\ros2-docker-workspace-vscode-plmrs`. The two independent
repositories must end with identical versions of the affected files.

## Validation

Update
`test_default_config_uses_fixed_priority_random_scene_layout()` in
`src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py`. Replace its
two-name configuration and selected-prefix assertions with five-name
assertions, and verify:

- the configuration contains the five fixed representatives in the required
  order;
- the scene still contains exactly six unique objects;
- the first five selected objects are the fixed representatives;
- all six objects retain the existing fixed placement-slot coordinates; and
- the sixth object is selected from outside the fixed set.

Add
`test_default_scene_fixed_objects_cover_all_grasp_categories()` to
`src/my_course_pkg/test/test_grasp_selector.py`. It loads the simulator YAML,
resolves the five configured names through `GRASP_CATEGORY_BY_OBJECT`, and
verifies the resulting category set is exactly:

- `cylindrical_can`
- `banana`
- `round_top`
- `box`
- `tool_top`

This assertion protects the one-representative-per-category contract without
adding category-aware selection behavior to the simulator runtime.

In the active container, resolve each representative through
`YCB_GRASP_NAME_MAP` and verify that its directory exists below `GRASP_ROOT`.
This is an environment verification step rather than a host-side unit-test
requirement because `/home/ws/grasps` is container-local.

Run the focused `test_populate_scene_selection.py` suite plus the category
coverage assertion. No live simulation, perception pipeline, or robot motion
is required for this configuration change.

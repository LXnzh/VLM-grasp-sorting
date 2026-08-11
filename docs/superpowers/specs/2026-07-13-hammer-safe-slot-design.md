# Hammer Safe Slot Design

## Goal

Prevent the fixed hammer from being ejected during MuJoCo startup while
preserving the existing object order and the other five placement slots.

## Root Cause

At the current fifth slot `[-0.55, 0.20, 0.0]`, the long hammer collision
geometry penetrates `banana_collision_5` by about `4.56 mm`. The first physics
step gives hammer a large upward and rotational velocity, ejecting it from the
table.

## Change

Change only the fifth entry in `placement_slots`:

```yaml
# Before
- [-0.55, 0.20, 0.0]

# After
- [-0.55, 0.30, 0.0]
```

Keep all other slot coordinates, `fixed_object_names`, object order,
orientations, and `random_object_count: 6` unchanged.

The active implementation workspace is
`E:\IFL\ros2-docker-workspace-vscode-plmrs-grasp-stable`. Apply and verify the
same YAML and test changes there first, then synchronize them to
`E:\IFL\ros2-docker-workspace-vscode-plmrs`.

## Validation

Update the expected slot list in
`test_default_config_uses_fixed_priority_random_scene_layout()`.

Run the complete focused scene-selection test file. Then run an isolated,
headless MuJoCo replay for at least 500 physics steps and verify:

- hammer has no initial collision with banana or another scene object;
- hammer remains on the table;
- hammer XY drift remains below `1 mm`; and
- no simulation, perception pipeline, ROS motion command, or robot grasp is
  executed as part of this configuration validation.

The approved candidate `[-0.55, 0.30, 0.0]` already completed a 500-step
exploratory replay with no object-object contact and about `0.042 mm` XY drift.

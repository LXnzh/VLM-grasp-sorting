# Interactive Mix/Random Scene Modes Design

## Goal

Preserve the current stable scene as `mix` while adding a `random` scene that
selects one object from each of the five existing grasp categories. Every
normal invocation of
`ros2 launch ifl_air_ur_launch cell_small_full_mujoco_moveit.launch.py` asks
which scene to start before any launch children run.

## Required Behavior

### `mix`

`mix` is the current behavior and must remain unchanged:

1. `tomato_soup_can`
2. `banana`
3. `apple`
4. `foam_brick`
5. `hammer`
6. one random object not already selected

The six selected objects keep the existing six fixed placement slots in this
order.

### `random`

`random` selects the first five objects independently from the existing grasp
categories, in slot order:

1. one cylindrical can: `tomato_soup_can` or `tuna_fish_can`
2. `banana`
3. one round-top object: `apple`, `lemon`, `peach`, `pear`, `orange`, `plum`,
   `baseball`, `tennis_ball`, or `racquetball`
4. one box object: `pudding_box`, `gelatin_box`, `sponge`, `foam_brick`, or
   `rubiks_cube`
5. `hammer`

The sixth object is sampled from the complete configured object pool after
excluding the first five selections. It may belong to any category but must
not duplicate an object already present in the scene.

`banana` and `hammer` are singleton categories, so their identities and their
validated slot assignments remain fixed in both modes.

## Architecture

Keep the current `select_scene_objects()` implementation as the `mix` path so
its established behavior and tests remain intact. Add a separate category-aware
selection function for `random`, plus a small mode dispatcher used by the
MuJoCo interface.

Store the ordered category membership in `base_env.yaml`, alongside a default
`scene_mode: mix`. Pass the mode and category configuration through
`ros2_main.py`/`main.py` into `MuJoCoInterface`. Direct simulator entrypoints
therefore remain backward-compatible and use `mix` unless explicitly
overridden.

The selector returns objects in placement-slot order. The existing
`assign_placement_slots()` function remains the only component that applies
coordinates, and no placement coordinate changes are in scope.

## Launch Interaction

Add a `scene_mode` launch argument with default value `prompt`. An early launch
setup action resolves it before starting the simulator, robot-state publisher,
gripper adapter, MoveIt, or arm API.

With the default command, the terminal displays:

```text
Select scene mode [random/mix]:
```

Input is trimmed and case-insensitive. Invalid interactive input prints a clear
message and asks again without starting any child process. Ctrl+C or closed
stdin terminates the launch cleanly.

Automation may bypass the prompt with either of these explicit forms:

```bash
ros2 launch ifl_air_ur_launch cell_small_full_mujoco_moveit.launch.py scene_mode:=mix
ros2 launch ifl_air_ur_launch cell_small_full_mujoco_moveit.launch.py scene_mode:=random
```

Any other explicit value fails before child processes start. The validated
mode is forwarded to Hydra as a simulator configuration override.

## Validation And Failure Behavior

Before category-aware selection, validate that:

- the mode is supported;
- every required category is present and non-empty;
- every category member exists in the configured object pool;
- an object is not assigned to more than one category; and
- at least one unselected object remains for slot 6.

Configuration errors raise a clear exception instead of silently falling back
to `mix` or returning fewer than six objects. Startup logs report the selected
mode and the six object names in placement-slot order.

Random selection occurs once per complete launch. `/reset_sim` continues to
restore the current scene and does not reroll object identities.

## Test Plan

Add deterministic unit coverage with seeded random generators for:

- exactly one selection from each category;
- `banana` at selection/slot 2 and `hammer` at selection/slot 5;
- a non-duplicating sixth selection from the remaining full pool;
- valid results across multiple seeds;
- unchanged `mix` selection behavior;
- invalid modes, empty categories, unknown objects, duplicate category
  membership, and no remaining sixth-object candidate;
- interactive `random`/`mix` parsing, invalid-input retry, clean interruption,
  and explicit-argument prompt bypass; and
- configuration propagation from the simulator entrypoint to
  `MuJoCoInterface`.

Run the simulator selection tests, relevant launch/helper tests, Python compile
checks, and builds for the affected ROS packages. No simulator launch, arm
motion, or grasp execution is part of automated verification.

## Out Of Scope

This change does not modify grasp profiles, grasp planning/execution,
perception, object geometry, fixed slot coordinates, collision-clearance
rules, robot controllers, or `/reset_sim` semantics.

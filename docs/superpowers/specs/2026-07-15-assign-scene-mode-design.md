# Assign Scene Mode Design

## Goal

Extend the approved and implemented `mix`/`random` startup selection with a
third `assign` mode. The user may require zero to six configured objects in a
specific slot order. Any unassigned slots are filled randomly from the
remaining object pool without duplicates.

`mix` and `random` behavior, placement coordinates, reset behavior, grasp
logic, perception, and robot control remain unchanged.

## Launch Interaction

The normal full launch first asks:

```text
Select scene mode [mix/random/assign]:
```

`mix` and `random` retain their current startup paths. Selecting `assign`
causes the launch process to read the configured `objects` list from
`src/ifl_air_mujoco_sim/env/config/base_env.yaml`, display all 18 available
names, and then ask:

```text
Enter up to 6 required objects, comma-separated (blank = all random):
```

The launch reads and validates this input before starting the simulator,
robot-state publisher, gripper adapter, MoveIt, or arm API. Passing
`scene_mode:=assign` skips only the first mode question; it still asks for the
required-object line.

## Input Rules

- Input is one comma-separated line.
- Whitespace around each comma-separated name is ignored.
- Names are case-insensitive and normalized to the configured lowercase name.
- The input order maps directly to placement slots 1 through 6.
- Zero names are valid: a blank line requests six random objects.
- One through five names are kept first and the remaining slots are filled
  randomly.
- Six names produce exactly the requested scene with no random fill.
- More than six names are invalid.
- Unknown names and duplicate names are invalid.
- A non-empty line containing an empty entry, such as `banana,,apple`, is
  invalid rather than silently repaired.

On invalid interactive input, the launch displays a clear reason and the valid
object list, then asks for the entire line again. No child process starts while
the prompt is unresolved. Ctrl+C propagates normally; closed stdin raises a
clear startup error.

## Object-List Source Of Truth

The launch must not contain a second hard-coded list of object names. It reads
the `objects` entries from the existing YAML configuration using PyYAML. The
launch package declares the matching runtime dependency. A missing, malformed,
empty, or duplicate-name configuration fails before startup.

The current configured pool contains exactly these 18 objects:

```text
tomato_soup_can, tuna_fish_can, pudding_box, gelatin_box, banana,
apple, lemon, peach, pear, orange, plum, sponge, hammer, baseball,
tennis_ball, racquetball, foam_brick, rubiks_cube
```

## Simulator Selection

Add `assigned_object_names: []` to `base_env.yaml` and pass it through
`ros2_main.py`/`main.py`, `UR10eRos2Interface`, and `MuJoCoInterface` alongside
the existing scene mode.

Add a dedicated `select_assigned_scene_objects()` function. It independently
validates the assigned names against the object pool, enforces the six-object
limit and uniqueness, retains the assigned objects in input order, and uses the
injected random generator to sample the remaining objects without replacement.

Extend the mode dispatcher and normalizer from `mix`/`random` to
`mix`/`random`/`assign`:

- `mix` calls the existing `select_scene_objects()` unchanged;
- `random` calls the existing category-aware selector unchanged; and
- `assign` calls the new assigned-object selector.

The launch serializes the already validated names as a Hydra list override,
for example:

```text
scene_mode=assign assigned_object_names=[banana,apple,foam_brick]
```

The simulator logs the selected mode and final six names in slot order.
`/reset_sim` restores this selected scene and does not reroll identities.

## Failure Behavior

Both launch and simulator validate the assigned list. Launch validation gives
the interactive user an immediate retry; simulator validation prevents direct
Hydra invocation from bypassing safety checks. Invalid configuration never
falls back silently to another mode or removes an invalid entry.

If the configured pool contains fewer than six unique objects, `assign` fails
instead of returning a short scene. This mode always produces exactly six
objects when startup succeeds.

## Test Plan

Add deterministic tests for:

- six assigned objects preserving exact input and slot order;
- one through five assigned objects followed by unique random fill;
- an empty assigned list producing six unique random objects;
- random-fill validity across multiple seeds;
- unknown, duplicate, and more-than-six simulator assignments failing closed;
- case-insensitive comma parsing with optional surrounding spaces;
- blank input;
- unknown names, duplicate names, too many names, and empty entries causing a
  complete interactive retry;
- valid-object loading from the YAML source of truth and malformed/duplicate
  YAML failure;
- `scene_mode:=assign` still invoking the object prompt;
- launch serialization of the validated Hydra list;
- unchanged `mix` and `random` behavior; and
- configuration propagation to `MuJoCoInterface`.

Run the combined simulator/launch regression, grasp-selector compatibility
tests, Python compilation, Hydra configuration inspection, and the
`ifl_air_ur_launch` symlink build. Confirm installed `--show-args` includes
`assign` in the scene-mode description. Do not automatically start the
simulator or any robot motion.

## Workspace Synchronization

The active Docker container currently mounts the sibling `-grasp-stable`
workspace. Before copying changes, compare every affected runtime/test file and
apply only the approved feature delta when no unrelated overlap exists. Keep
the two workspaces' affected files equivalent apart from line endings, while
leaving all unrelated dirty files untouched.

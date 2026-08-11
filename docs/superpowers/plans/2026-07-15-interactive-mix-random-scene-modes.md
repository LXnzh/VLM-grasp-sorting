# Interactive Mix/Random Scene Modes Implementation Plan

**Goal:** Preserve the current `mix` scene and add an interactive `random`
scene that samples one object per grasp category, keeps banana/hammer fixed,
and adds a non-duplicating sixth random object.

## Step 1: Selection tests

Modify `src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py` first.
Add deterministic tests for category order, singleton banana/hammer, the sixth
remaining-pool selection, multi-seed validity, unchanged `mix` dispatch, and
fail-closed invalid configurations.

Run the focused test and require the new tests to fail before implementation:

```bash
PYTHONPATH=/home/ws/src/ifl_air_mujoco_sim \
  /usr/bin/python3 -m pytest \
  src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py -q
```

## Step 2: Category selector and configuration

Modify:

- `src/ifl_air_mujoco_sim/env/config/base_env.yaml`
- `src/ifl_air_mujoco_sim/env/utils/populate_scene.py`

Keep `select_scene_objects()` unchanged. Add a separate category-aware random
selector and a strict `mix`/`random` dispatcher. Define the ordered five
categories in YAML and keep `scene_mode: mix` as the direct-entrypoint default.

Run the focused selection test until it passes.

## Step 3: Simulator propagation

Modify:

- `src/ifl_air_mujoco_sim/ros2_main.py`
- `src/ifl_air_mujoco_sim/main.py`
- `src/ifl_air_mujoco_sim/env/ros2_interface.py`
- `src/ifl_air_mujoco_sim/env/mjcontrol_interface.py`

Pass `scene_mode` and `scene_object_categories` into `MuJoCoInterface`, use the
mode dispatcher before slot assignment, and log mode plus selected object
order. Do not modify reset, placement, geometry, perception, or control paths.

## Step 4: Launch prompt tests and implementation

Add `src/ifl_air_ur_launch/test/test_scene_mode_launch.py` first. Cover explicit
mode normalization, prompt selection, invalid-input retry, EOF, interruption,
and invalid explicit values.

Modify
`src/ifl_air_ur_launch/launch/cell_small_full_mujoco_moveit.launch.py` to add a
`scene_mode` argument whose default is `prompt`. Resolve it in an early
`OpaqueFunction`, before all child actions, and append the validated Hydra
override to the MuJoCo command.

Run:

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m pytest \
  src/ifl_air_ur_launch/test/test_scene_mode_launch.py -q
```

## Step 5: Verification and handoff

Run focused and related simulator tests, Python compilation, and builds:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH=/home/ws/src/ifl_air_mujoco_sim:/home/ws/src/ifl_air_mujoco_sim/.venv/lib/python3.10/site-packages:$PYTHONPATH \
/usr/bin/python3 -m pytest \
  src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py \
  src/ifl_air_mujoco_sim/test/test_scene_clearance_bounds.py \
  src/ifl_air_mujoco_sim/test/test_scene_clearance_publisher.py -q

/usr/bin/python3 -m py_compile \
  src/ifl_air_mujoco_sim/env/utils/populate_scene.py \
  src/ifl_air_mujoco_sim/env/mjcontrol_interface.py \
  src/ifl_air_mujoco_sim/env/ros2_interface.py \
  src/ifl_air_mujoco_sim/ros2_main.py \
  src/ifl_air_mujoco_sim/main.py \
  src/ifl_air_ur_launch/launch/cell_small_full_mujoco_moveit.launch.py

colcon build --packages-select ifl_air_ur_launch --symlink-install
```

Update `docs/agent_handoff.md` with implemented behavior, verification results,
and the two supported launch forms. Do not start the simulator or any robot
motion automatically.

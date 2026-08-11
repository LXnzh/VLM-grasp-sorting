# Assign Scene Mode Implementation Plan

**Goal:** Add `assign` to the existing uncommitted `mix/random` architecture so
the user can require zero to six objects in slot order and randomly fill the
remaining slots without duplicates.

## Step 1: Simulator tests first

Extend `src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py` with
failing tests for exact six-object assignment, partial assignment plus seeded
random fill, blank assignment, multiple seeds, dispatcher routing, and
fail-closed unknown/duplicate/too-many/too-small-pool cases. Assert the YAML
default contains `assigned_object_names: []`.

Run the focused test from a temporary Docker copy and require the new tests to
fail before implementation.

## Step 2: Launch tests first

Extend `src/ifl_air_ur_launch/test/test_scene_mode_launch.py` with failing tests
for:

- `assign` mode parsing and the updated mode prompt;
- YAML object-name loading, including malformed, empty, and duplicate config;
- comma parsing, optional whitespace, case normalization, and blank input;
- retry on unknown, duplicate, over-six, and empty entries;
- closed input and Ctrl+C behavior;
- `scene_mode:=assign` still resolving assigned objects; and
- Hydra serialization of the validated assigned list.

Run the focused launch test from the temporary Docker copy and require failure
before implementation.

## Step 3: Simulator implementation

Modify:

- `src/ifl_air_mujoco_sim/env/config/base_env.yaml`
- `src/ifl_air_mujoco_sim/env/utils/populate_scene.py`
- `src/ifl_air_mujoco_sim/env/mjcontrol_interface.py`
- `src/ifl_air_mujoco_sim/env/ros2_interface.py`
- `src/ifl_air_mujoco_sim/ros2_main.py`
- `src/ifl_air_mujoco_sim/main.py`

Add `assigned_object_names: []`, a strict six-object assigned selector, the
third dispatcher branch, and end-to-end configuration propagation. Keep the
existing `mix` and `random` functions unchanged.

## Step 4: Launch implementation

Modify:

- `src/ifl_air_ur_launch/launch/cell_small_full_mujoco_moveit.launch.py`
- `src/ifl_air_ur_launch/package.xml`

Load object names from the existing simulator YAML, add the second prompt and
retry loop, serialize only validated canonical names into a shell-quoted Hydra
list override, and declare the `python3-yaml` runtime dependency. Resolve all
interaction before returning the simulator process from the early
`OpaqueFunction`.

## Step 5: Synchronize and verify

Before changing the Docker-mounted sibling workspace, compare all affected
files and confirm there is no unrelated overlap. Apply the same runtime/test
delta there without touching unrelated dirty files.

Run in `/home/ws`:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH=/home/ws/src/ifl_air_mujoco_sim:/home/ws/src/ifl_air_mujoco_sim/.venv/lib/python3.10/site-packages:$PYTHONPATH \
/usr/bin/python3 -m pytest \
  src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py \
  src/ifl_air_mujoco_sim/test/test_scene_clearance_bounds.py \
  src/ifl_air_mujoco_sim/test/test_scene_clearance_publisher.py \
  src/ifl_air_ur_launch/test/test_scene_mode_launch.py -q

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  src/my_course_pkg/test/test_grasp_selector.py -q

/usr/bin/python3 -m py_compile \
  src/ifl_air_mujoco_sim/env/utils/populate_scene.py \
  src/ifl_air_mujoco_sim/env/mjcontrol_interface.py \
  src/ifl_air_mujoco_sim/env/ros2_interface.py \
  src/ifl_air_mujoco_sim/ros2_main.py \
  src/ifl_air_mujoco_sim/main.py \
  src/ifl_air_ur_launch/launch/cell_small_full_mujoco_moveit.launch.py

colcon build --packages-select ifl_air_ur_launch --symlink-install
ros2 launch ifl_air_ur_launch \
  cell_small_full_mujoco_moveit.launch.py --show-args
```

Also inspect Hydra config with `scene_mode=assign` and a sample assigned list.
Do not start the simulator or execute robot motion automatically. Update both
handoffs with behavior, verification, and the next live user check.

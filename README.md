# grasp_stable

Pure stable-grasp workspace: MuJoCo scene generation, perception, grasp-strategy
selection, guarded grasp, and lift. No food/non-food sorting, bin localization,
or GUI-pipeline launchers.

Primary entry point:

```bash
ros2 launch ifl_air_ur_launch experiment_session.launch.py
```

Do **not** use `gui_manager.py` for this product.

## Prerequisites

1. Open this repository in the Dev Container (`/home/ws`).
2. Build once:

```bash
cd /home/ws
colcon build --symlink-install
source install/setup.bash
```

3. Create the MuJoCo package venv if it is missing:

```bash
cd /home/ws/src/ifl_air_mujoco_sim
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

4. Install local runtime data (Git-excluded; not in this repo):

- YCB meshes: `/home/ws/src/my_course_pkg/YCB_Dataset/ycb`
- Grasp library: `/home/ws/grasps`

Set `export MUJOCO_GL=egl` when using GPU rendering (usually already in the
container `~/.bashrc`).

## Run an experiment session

From a sourced Dev Container terminal:

```bash
source /home/ws/setup_ros.bash   # or: source install/setup.bash
ros2 launch ifl_air_ur_launch experiment_session.launch.py
```

The launch will:

1. Ask for the **VLM API key** (hidden input), unless `VLM_API_KEY` is already
   exported.
2. Print the **16 available objects** and ask you to generate the scene.
3. Start MuJoCo + MoveIt for the assigned scene.
4. Run the experiment supervisor. Each trial asks for an **Instruction**, then
   runs perception (`pipeline`) and grasp (`grasp_demo`).

### Scene generation (objects)

Prompt looks like:

```text
Available objects (16): tomato_soup_can, gelatin_box, banana, apple, lemon,
peach, pear, orange, plum, sponge, hammer, baseball, tennis_ball, racquetball,
foam_brick, rubiks_cube
Enter up to 6 required objects, comma-separated (blank = all random):
```

| Input | Result |
| --- | --- |
| blank Enter | sample 6 unique objects from the 16-object pool |
| `apple, banana` | keep those first, then randomly fill up to 6 |
| up to 6 names | use those names (must be in the pool) |

Excluded from scene generation: `tuna_fish_can`, `pudding_box`.

### Instruction (perception target)

After the scene is up and the trial has reset, the supervisor prompts:

```text
Instruction:
```

Type a natural-language instruction for the object to grasp, for example:

- `pick up the apple`
- `grasp the banana`
- `blue racquetball` / `racquetball` / `blue ball` (same racquetball alias)

Empty input is rejected. Perception uses the instruction to select the mask and
estimate pose; then `grasp_demo` executes the grasp.

After a trial finishes (or fails), press Enter for the next trial, or `q` to
quit. Session logs are written under `/tmp/my_course_experiment_sessions`.

### After `/reset_sim`

Old perception output is stale. Always run a fresh instruction / perception
pass before grasping again.

## Manual commands (optional)

If you are not using the experiment-session supervisor:

```bash
# Perception only (prompts Instruction:)
ros2 run my_course_pkg pipeline

# Grasp using the latest perception result
ros2 run my_course_pkg grasp_demo

# Soft reset of the MuJoCo scene
ros2 service call /reset_sim std_srvs/srv/Trigger
```

Full stack without the interactive session:

```bash
ros2 launch ifl_air_ur_launch cell_small_full_mujoco_moveit.launch.py
```

## Product boundary

This repository performs scene generation, perception, strategy selection,
guarded grasp, lift, and the configured pure-grasp completion path.

It does **not** include:

- food / non-food classification
- RGB-D sorting-bin localization
- classified placement / sorting tasks
- the GUI / tracking / PBVS product line

## Useful packages

| Package | Role |
| --- | --- |
| `my_course_pkg` | Perception pipeline, grasp demo, experiment session |
| `ifl_air_mujoco_sim` | MuJoCo simulation backend |
| `ifl_air_ur_launch` | Launch files (`experiment_session`, full cell) |
| `arm_api2` / `arm_api2_py` | MoveIt robot interface |

## Notes

- Run Git on the Windows host; run ROS / MuJoCo / tests inside the Dev Container.
- Do not run two Dev Containers that share host networking for this stack at once.
- Do not reuse `build/`, `install/`, `log/`, or experiment artifacts from another
  worktree.

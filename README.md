# VLM Grasp Sorting

End-to-end MuJoCo manipulation for natural-language object selection, dynamic
RGB-D tracking, stable grasp execution, and food/non-food placement.

The product path is:

```text
GUI text or browser microphone
  -> synchronized home overview RGB-D
  -> VLM target selection + food/non-food classification
  -> freeze world-frame placement target
  -> SAM2 lock + PBVS follow
  -> continuous stop/stability gate
  -> FoundationPose 6D pose
  -> stable grasp selection/planning/execution
  -> place, release, retreat, return home
```

The grasp prefix through lift is the validated `grasp_stable_pure` behavior.
Sorting only supplies an explicit post-lift placement target. Food uses a
randomized `food_bin` located from the initial overview RGB-D frame; non-food
uses the configured world-frame drop point.

## Quick start

Use the repository Dev Container. ROS 2 and MuJoCo commands are intended to run
inside the container at `/home/ws`; use Git on the Windows host.

The Dev Container installs ROS dependencies and
[requirements.txt](requirements.txt). Build the workspace once:

```bash
cd /home/ws
colcon build --symlink-install
source install/setup.bash
```

Configure the OpenAI-compatible VLM credential without committing it:

```bash
export VLM_API_KEY="..."
```

The defaults target the KIT services used by this project. Override them when
running different service instances:

```bash
export VLM_BASE_URL="https://ki-toolbox.scc.kit.edu/api/v1"
export VLM_MODEL="azure.gpt-5-mini"
export SAM2_URL="http://172.22.222.226:5000/predict"
export FOUNDATIONPOSE_URL="http://172.22.222.220:5001"
```

Start the desktop workflow:

```bash
cd /home/ws
source install/setup.bash
python3 gui_manager.py
```

In the GUI:

1. Click **Start Food-Sorting Simulation** and wait for the robot/camera logs.
2. Type an instruction, or enable microphone input and use the browser recorder.
3. Click **Start Complete Grasp-and-Sort Flow**.
4. The object may move only after `TARGET_LOCKED`. Keep it still once the log
   reports the stop/stability window.

The API key field is passed to child processes through the environment and is
not printed in the command log. Browser voice capture uses
`http://localhost:8765` and the configured OpenAI-compatible transcription
service.

## CLI workflow

Start a non-interactive randomized sorting scene:

```bash
export MY_COURSE_SINGLE_BIN_MODE=1
ros2 launch ifl_air_ur_launch \
  cell_small_full_mujoco_moveit.launch.py scene_mode:=random
```

在没有显示器的机器或 CI 容器中，加上 `sim_headless:=true`。正常交互运行不加该参数，MuJoCo 窗口默认保持开启。

In a second sourced terminal:

```bash
ros2 run my_course_pkg pbvs_sorting_grasp \
  --instruction "pick up the apple"
```

Browser microphone input is also available directly:

```bash
ros2 run my_course_pkg pbvs_sorting_grasp \
  --voice --voice-input browser --voice-language en
```

Soft-reset the simulation with:

```bash
ros2 service call /reset_sim std_srvs/srv/Trigger '{}'
```

A reset invalidates perception output. Always start a fresh complete task after
resetting.

## Supported objects

Exactly these 16 YCB objects are accepted by the scene, GUI/VLM path,
FoundationPose asset resolver, and grasp library:

```text
tomato_soup_can, gelatin_box, banana, apple, lemon, peach, pear, orange,
plum, sponge, hammer, baseball, tennis_ball, racquetball, foam_brick,
rubiks_cube
```

`tuna_fish_can` and `pudding_box` are deliberately unsupported. Overrides
or stale inputs naming either object fail before planning.

The numbered YCB runtime assets are under
`src/my_course_pkg/YCB_Dataset/ycb`; the matching grasp library is under
`grasps`.

## Safety and failure behavior

This repository is currently accepted for MuJoCo simulation, not real robot
operation. The complete task fails closed before grasp planning when any of the
following is missing or invalid:

- supported target identity;
- exact `food` or `non_food` VLM classification;
- synchronized overview RGB-D and camera transform;
- visible food-bin localization for a food target;
- valid SAM2 mask, PBVS tracking, or continuous stability window;
- FoundationPose result, stable grasp candidate, or typed placement target.

The food-bin target is observed at the home overview pose and frozen in the
world frame before PBVS begins. It is never inferred from the close-up stable
grasp frame.

## Verification

Run the core package tests:

```bash
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
source /opt/ros/humble/setup.bash
source /home/ws/install/setup.bash

cd /home/ws/src/my_course_pkg
python3 -m pytest -q test -k 'not copyright and not flake8 and not pep257'

cd /home/ws/src/ifl_air_mujoco_sim
MUJOCO_GL=egl python3 -m pytest -q test
```

The MuJoCo suite includes a real home-camera field-of-view preflight and
rendered overview RGB-D food-bin localization.

## Main modules

| Component | Responsibility |
| --- | --- |
| `gui_manager.py` | Text/voice interaction, launch control, live state/log |
| `tasks/tracking` | Overview selection, SAM2 lock, tracking, stability, grasp handoff |
| `tasks/pbvs` | Camera-relative follow control and safe stop |
| `tasks/sorting` | Strict classification and immutable placement target |
| `grasp` | Stable grasp selection, planning, guarded execution |
| `ifl_air_mujoco_sim` | Canonical six-object scene and randomized `food_bin` |

YCB model data originates from the
[YCB Benchmarks project](https://www.ycbbenchmarks.com/). Review upstream data
terms when redistributing assets outside a research project.

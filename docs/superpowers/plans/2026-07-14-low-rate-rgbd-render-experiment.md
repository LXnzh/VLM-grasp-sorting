# Low-Rate RGB-D Render Experiment Implementation Plan

> Implement the approved design in
> `docs/superpowers/specs/2026-07-13-low-rate-rgbd-render-design.md`.
> This plan authorizes a configuration-only cadence experiment and read-only
> ROS validation. It does not authorize `grasp_demo`, arm motion, Cartesian
> servo, or gripper commands.

## Goal

Change only the active `-grasp-stable` workspace from `render_fps: 20` to
`render_fps: 0.2`, then prove in two independent simulator starts that RGB-D
remains usable and `/joint_states`, `/scene_description`, and
`/scene_clearance_bounds` each sustain at least `18 Hz`.

## Constraints

- Preserve every unrelated dirty or untracked change in both workspaces.
- Do not copy the full YAML or test file between workspaces.
- Keep one `camera_orbbec`, 1280x720, depth enabled, and both pointcloud flags
  disabled.
- Do not change simulator code, perception code, safety margins, scene objects,
  slots, controller parameters, or the 20 Hz state publish target.
- Do not use the exposed historical VLM credential.
- Do not run A1/A3 until both live cadence trials pass.

## Task 1: Confirm Stage 1 Baseline

**Active files:**

- `src/ifl_air_mujoco_sim/env/config/base_env.yaml`
- `src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py`

Confirm the active configuration and test already require:

```yaml
camera_names: ["camera_orbbec"]
camera_size: [1280, 720]
render_fps: 20
enable_pointcloud_camera1: false
enable_pointcloud_camera2: false
enable_depth_camera1: true
enable_depth_camera2: false
```

Record the active launch, `ros2_main.py`, and `moveit2_iface` PIDs before any
restart. Do not change any Stage 1 field other than `render_fps`.

## Task 2: TDD The Configuration Change

1. Change only the active config-test assertion from `render_fps == 20` to
   `render_fps == 0.2`.
2. Run the focused config test and require an assertion failure showing the
   YAML still contains `20`.
3. Change only the active YAML line to `render_fps: 0.2`.
4. Rerun the focused config test and require a pass.
5. Confirm the original workspace YAML remains unchanged; the approved design
   intentionally limits this experiment to the active runtime workspace.

## Task 3: Offline Regression

Use the container's MuJoCo environment only for simulator tests:

```bash
cd /home/ws
source /opt/ros/humble/setup.bash

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH=/home/ws/src/ifl_air_mujoco_sim:/home/ws/src/ifl_air_mujoco_sim/.venv/lib/python3.10/site-packages:$PYTHONPATH \
/usr/bin/python3 -m pytest \
  src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py \
  src/ifl_air_mujoco_sim/test/test_scene_clearance_bounds.py \
  src/ifl_air_mujoco_sim/test/test_scene_clearance_publisher.py -q
```

Require all currently collected simulator/scene-clearance tests to pass and
run `git diff --check` on the two active files.

## Task 4: Independent Live Cadence Trial 1

1. Send SIGINT to the existing ROS launch process and wait for its simulator
   and MoveIt children to exit.
2. Verify no stale `ros2_main.py` remains.
3. Start the standard full MuJoCo/MoveIt launch with output captured to
   `/tmp/low_rate_rgbd_trial1_launch.log`.
4. Wait for the required nodes and five topics to exist:
   - `/joint_states`;
   - `/scene_description`;
   - `/scene_clearance_bounds`;
   - `/camera_orbbec/color/image_raw`;
   - `/camera_orbbec/depth/image_raw`.
5. Confirm no `/camera_third_person/*` or pointcloud publisher exists.
6. Observe at least 35 seconds and require at least six RGB and six depth
   messages with matching cadence. Verify RGB is 1280x720 `bgr8`, depth is
   1280x720 `32FC1`, and timestamps advance.
7. In the same launch, measure each state topic for at least 30 seconds. Require
   every reported average to be `>=18 Hz`.
8. Read one fresh bounds array and require six finite, positive-scale markers.
   Confirm `/scene_clearance_bounds` remains reliable/volatile.
9. Inspect the launch log for bounds construction/publication errors and record
   loop/camera timing evidence.

Do not run pipeline, MoveIt goals, or any motion command in this trial.

## Task 5: Independent Live Cadence Trial 2

Stop Trial 1 cleanly, verify its processes exited, and repeat every Task 4 step
in a new launch. Save its output separately as
`/tmp/low_rate_rgbd_trial2_launch.log`.

Both live trials must pass. Any failed trial resets the cadence success count.

## Task 6: Pass Or Roll Back

### If both trials pass

- Keep `render_fps: 0.2` in the active workspace.
- Record both measured topic rates and RGB-D counts in both handoffs.
- Hand off the next user-driven sequence:
  1. A1: two independent reset + fresh Apple pipeline trials;
  2. A3: two independent reset + fresh Apple pipeline + `grasp_plan_only`
     trials;
  3. only after both stages pass, consider Phase B.

The user must enter a rotated VLM key privately in their terminal. Do not store
or repeat it.

### If either trial fails

- Restore the active YAML and config-test assertion to `20`.
- Rerun the focused test and `git diff --check`.
- Save all topic, RGB-D, bounds, process, and launch-log evidence.
- Do not run A1, A3, or Phase B.
- Start a separate design for replacing the fixed `18 Hz` number with measured
  functional control-health gates. Do not proceed with no replacement gate.

## Definition Of Done

- Only the active YAML `render_fps` and its focused assertion change.
- Focused RED/GREEN behavior is demonstrated.
- Simulator/scene-clearance regression passes.
- Two independent live launches each pass all RGB-D, bounds, QoS, process, and
  `>=18 Hz` state-topic gates, or the config is restored after failure.
- No pipeline, MoveIt goal, arm, Cartesian, or gripper command runs during the
  cadence experiment.
- Handoffs state the measured result and the next safe action.

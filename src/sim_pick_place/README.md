# sim_pick_place

Lightweight ROS 2 package providing a pick-and-place simulation node and utilities.
Works with a simulated robotic arm for example in Mujoco (https://gitlab.kit.edu/kit/ifl/gruppen/air/ifl_air_mujoco_sim).

Quick start

- Build and install (in a ROS 2 workspace):

```bash
# from the workspace root
colcon build --packages-select sim_pick_place
source install/setup.bash
```

- Run the node (recommended):

```bash
ros2 run sim_pick_place sim_pick_place_node
```

- Run directly for quick testing (no colcon):

```bash
python -m sim_pick_place.sim_pick_place_node
```

Tests

```bash
pytest -q
```

Notes

- Package type: ament_python (see `package.xml` / `setup.py`).
- The node expects a `MarkerArray` on `/scene_description` and uses TF2 and an Arm API client.

License

See `LICENSE` for details.

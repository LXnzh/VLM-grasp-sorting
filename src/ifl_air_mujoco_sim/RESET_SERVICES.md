# ROS2 Reset Services

The MuJoCo UR10e simulation provides two ROS2 services for resetting the simulation state.

## Services

### 1. `/reset_sim` - Default Reset

Resets the simulation to the default initial configuration defined in the MuJoCoInterface.

**Usage:**
```bash
ros2 service call /reset_sim std_srvs/srv/Trigger
```

**Response:**
- `success`: Boolean indicating if reset was successful
- `message`: Status message


### 2. `/reset_sim_custom` - Custom Reset

Resets the simulation with custom end-effector pose and object positions/orientations. The custom configuration is specified via a ROS2 parameter.

**Usage:**

First, set the custom reset specification parameter:
```bash
ros2 param set /mujoco_ur10e_interface reset_spec 'string:{
  "robot": {
    "joint_positions": [-0.5, -1.2, -1.5, -1.8, 1.6, 2.8]
  },
  "objects": {
    "red_cube": {
      "position": [0.5, 0.2, 0.15],
      "orientation": [1.0, 0.0, 0.0, 0.0]
    },
    "blue_sphere": {
      "position": [0.3, -0.3, 0.1]
    }
  }
}'
```

Then call the service:
```bash
ros2 service call /reset_sim_custom std_srvs/srv/Trigger
```

**Parameter Format (`reset_spec`):**

The `reset_spec` parameter should be a JSON string with the following structure:

```json
{
  "robot": {
    "joint_positions": [j1, j2, j3, j4, j5, j6]
  },
  "objects": {
    "object_name": {
      "position": [x, y, z],
      "orientation": [w, x, y, z]
    }
  }
}
```

**Fields:**
- `robot.joint_positions` (optional): Array of 6 joint angles in radians for the UR10e arm
- `objects` (optional): Dictionary of object names to pose specifications
  - `position` (optional): [x, y, z] position in world coordinates
  - `orientation` (optional): [w, x, y, z] quaternion orientation

**Note:** All fields are optional. If a field is not specified, the corresponding state will use the default initial value.

## Examples

### Example 1: Reset to default
```bash
ros2 service call /reset_sim std_srvs/srv/Trigger
```

### Example 2: Reset with custom arm position only
```bash
ros2 param set /mujoco_ur10e_interface reset_spec 'string:{"robot": {"joint_positions": [0.0, -1.57, -1.57, -1.57, 1.57, 0.0]}}'

ros2 service call /reset_sim_custom std_srvs/srv/Trigger
```

### Example 3: Reset with custom object positions only
```bash
ros2 param set /mujoco_ur10e_interface reset_spec 'string:{
  "objects": {
    "red_cube": {
      "position": [0.6, 0.0, 0.2]
    },
    "blue_sphere": {
      "position": [0.4, 0.3, 0.15],
      "orientation": [0.9239, 0.0, 0.0, 0.3827]
    }
  }
}'

ros2 service call /reset_sim_custom std_srvs/srv/Trigger
```

### Example 4: Reset everything
```bash
ros2 param set /mujoco_ur10e_interface reset_spec 'string:{
  "robot": {
    "joint_positions": [-0.2345, -1.0715, -1.8688, -1.5812, 1.6339, 2.8947]
  },
  "objects": {
    "box_red": {
      "position": [0.5, 0.2, 0.15],
      "orientation": [1.0, 0.0, 0.0, 0.0]
    },
    "sphere_blue": {
      "position": [0.3, -0.3, 0.1],
      "orientation": [1.0, 0.0, 0.0, 0.0]
    }
  }
}'

ros2 service call /reset_sim_custom std_srvs/srv/Trigger
```

## Programmatic Usage (Python)

Please refer to the `examples/example_reset_services.py` script for a complete Python example of using the reset services programmatically.

## Notes

- Both services are thread-safe and can be called while the simulation is running
- The custom reset specification persists until changed or the node is restarted
- If the JSON in `reset_spec` is invalid, the service will return an error message
- Object names must match the names defined in your objects configuration
- Quaternion orientations should be normalized [w, x, y, z] format

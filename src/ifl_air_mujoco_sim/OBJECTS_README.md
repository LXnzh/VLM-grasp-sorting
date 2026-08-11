# Object Configuration for MuJoCo-Grasp Simulation

This document describes the new object configuration system that allows adding multiple types of objects to the MuJoCo simulation scene and publishing their descriptions via ROS2.

## Features

### 1. Configurable Objects in Scene
The simulation now supports adding multiple objects (cubes, spheres, cylinders) to the scene through configuration files instead of being hardcoded.

### 2. ROS2 Scene Description Publisher
The simulation publishes a real-time scene description containing object positions, sizes, and types as ROS2 `MarkerArray` messages on the `/scene_description` topic.

## Configuration

Objects are defined in the `base_env.yaml` configuration file under the `objects` section:

```yaml
objects:
  - name: "red_cube"
    type: "box"
    size: [0.04, 0.04, 0.04]  # [width, depth, height] in meters
    position: [-0.5, 0.0, 0.2]  # [x, y, z] in meters
    orientation: [1, 0, 0, 0]  # quaternion [w, x, y, z]
    color: [1.0, 0.0, 0.0, 1.0]  # RGBA
  - name: "blue_sphere"
    type: "sphere"
    size: [0.03]  # radius in meters
    position: [-0.3, 0.1, 0.2]
    orientation: [1, 0, 0, 0]
    color: [0.0, 0.0, 1.0, 1.0]
  - name: "green_cylinder"
    type: "cylinder"
    size: [0.02, 0.06]  # [radius, height] in meters
    position: [-0.4, -0.1, 0.2]
    orientation: [1, 0, 0, 0]
    color: [0.0, 1.0, 0.0, 1.0]
```

### Object Parameters

- **name**: Unique identifier for the object
- **type**: Object geometry type (`"box"`, `"sphere"`, `"cylinder"`)
- **size**: Dimensions in meters (format depends on type)
  - Box: `[width, depth, height]`
  - Sphere: `[radius]`
  - Cylinder: `[radius, height]`
- **position**: 3D position `[x, y, z]` in meters
- **orientation**: Quaternion `[w, x, y, z]` (normalized)
- **color**: RGBA color values `[r, g, b, a]` (0.0-1.0 range)

## ROS2 Integration

### Published Topics

- **`/scene_description`** (`visualization_msgs/MarkerArray`): Real-time scene description with object positions, sizes, and visual properties

### Message Format

The scene description is published as a `MarkerArray` containing `Marker` messages with the following properties:

- **header.frame_id**: `"base_link"`
- **namespace**: `"scene_objects"`
- **type**: Corresponding RViz marker type (CUBE, SPHERE, CYLINDER)
- **pose**: Current 3D position and orientation
- **scale**: Object dimensions for visualization
- **color**: RGBA color values

## Usage Examples

### 1. Basic Simulation (Headless)
```python
from env.mjcontrol_interface import MuJoCoInterface

objects_config = [
    {
        'name': 'test_cube',
        'type': 'box',
        'size': [0.05, 0.05, 0.05],
        'position': [-0.4, 0.0, 0.2],
        'orientation': [1, 0, 0, 0],
        'color': [1.0, 0.5, 0.0, 1.0]
    }
]

sim = MuJoCoInterface(
    model_path='models/ur10e_2f85/scene.xml',
    objects_config=objects_config,
    headless=True
)

# Get current scene description
scene_desc = sim.get_scene_description()
print(f"Scene contains {len(scene_desc)} objects")
```

### 2. ROS2 Interface with Scene Publishing
```bash
# Run the ROS2 interface (publishes scene description automatically)
python ros2_main.py

# In another terminal, listen to scene description
ros2 topic echo /scene_description
```

### 3. Visualizing in RViz
```bash
# Launch RViz and add a MarkerArray display
# Set the topic to: /scene_description
# Objects will appear as colored markers in the visualization
```

## Implementation Details

### Key Components

1. **`populate_scene.py`**: Generates MuJoCo XML with objects from configuration
2. **`mjcontrol_interface.py`**: Core simulation interface with scene description API
3. **`ros2_interface.py`**: ROS2 wrapper with MarkerArray publishing
4. **`base_env.yaml`**: Configuration file with object definitions

### Object Physics

- All objects have `mass="0.1"` by default
- Objects include a `freejoint` for 6-DOF movement
- Collision and visual geometries are automatically generated

### Coordinate Frame

- All positions are relative to the world frame
- Z-axis points upward
- The robot base is typically at origin (0, 0, 0)

## Troubleshooting

### Common Issues

1. **"repeated name" error**: Object names must be unique and not conflict with existing scene elements
2. **Objects not appearing**: Check position values - objects might be below the table or outside view
3. **ROS2 publishing issues**: Ensure all ROS2 dependencies are installed and sourced

### Debugging

- Enable verbose logging to see object creation messages
- Use the test script to verify configuration loading
- Check ROS2 topic list: `ros2 topic list | grep scene`
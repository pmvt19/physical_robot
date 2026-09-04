# MapBuilders


The `MapBuilder` classes provide an easy to use interface for building maps of an environment. The SunoBot code provides 3 variants of MapBuilders: `MapBuilder`, `AdvancedMapBuilder`, and `SemanticMapBuilder` classes. Intuitively, these variants build a `Map`, `AdvancedMap`, or `SemanticMap` respectively.

# MapBuilder Class

The `MapBuilder` class coordinates a robotic system and its LiDAR sensor data to construct, update, and track a spatial map over time. It maintains the robot's current state, trajectories, and handles map updates per time step.

## Table of Contents
- [Initialization](#initialization)
- [Methods](#methods)
- [Usage Example](#usage-example)

---

## Initialization

```python
def __init__(self, robot, map_resolution=10.0, manual_lidar_verification=False)
```

Initializes the map building coordinator.

### Parameters
| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `robot` | `Robot` | *Required* | An instance of the `Robot` class providing movement and sensor interfaces. |
| `map_resolution` | `float` | `10.0` | The grid resolution scale for the generated map. Units in millimeters. |
| `manual_lidar_verification` | `bool` | `False` | If `True`, prompts manual user verification for every LiDAR reading. Mainly used for debugging purposes. |

### Internal State Attributes
* **`self.robot_state`** (`np.ndarray`): A 3-element array `[x, y, theta]` tracking the robot's current pose. Starts at `[0.0, 0.0, 0.0]`.
* **`self.map`** (`Map`): The underlying map object initialized with the given resolution.
* **`self.robot_trajectory`** (`list[np.ndarray]`): A history list recording the robot's state at every step.

---

## Methods

### `init()`
Performs the initial LiDAR scan to establish and initialize the map's baseline geometry.
* **Returns:** `None`

### `step(m)`
Executes a single iteration of the localization and mapping loop. It predicts the new state based on movement, reads the sensor, and refines the map.
* **Parameters:** `m`: Movement/Odometry data passed to the robot's state prediction model.
* **Returns:** `None`

### `get_map()`
* **Returns:** `Map` — The current state of the map object.

### `get_robot_state()`
* **Returns:** `np.ndarray` — The current `[x, y, theta]` pose of the robot.

### `get_robot_trajectory()`
* **Returns:** `list[np.ndarray]` — A chronological list of all historic robot states.

### `show()`
Renders a visual representation of the current map using `matplotlib`.
* **Returns:** `None`

---

## Usage Example

```python
from physical_robot.robot import Robot
from physical_robot.map_builder import MapBuilder

# 1. Setup the robot and builder
robot = Robot()
map_builder = MapBuilder(robot=robot, map_resolution=12.5)

# 2. Baseline the map
map_builder.init()

while True:
    # 3. Get Motion Commands from User
    motion_command = robot.request_motion_command_from_user()
    if motion_command[0] == '': # No Motion Command
        break

    # 4. Move the Robot
    m = robot.command_motion(motion_command)

    # 5. Step The Map Builder
    map_builder.step(m)

    # 6. Fetch results or visualize
    current_pose = map_builder.get_robot_state()
    print(f"Robot is at: {current_pose}")

    # 7. Visualize Current Map
    map_builder.show()

# 8. Save the Map
map_builder.get_map().save(map_save_dir="saves/maps")
```

## AdvancedMapBuilder

`AdvancedMapBuilder` inherits from `MapBuilder` and preserves the same high-level map-building workflow while using an `AdvancedMap` implementation for richer geometry and exploration features.

### Key differences from `MapBuilder`
- Inherits all public methods from `MapBuilder`: `init()`, `step(m)`, `get_map()`, `get_robot_state()`, `get_robot_trajectory()`, and `show()`.
- Replaces the base `Map` with `AdvancedMap` during initialization.
- Gains advanced map semantics such as probabilistic occupancy, obstacle inflation, and frontier candidate generation.
- Uses the same LiDAR-based scan initialization and update loop as `MapBuilder`.

### Initialization
```python
map_builder = AdvancedMapBuilder(robot=robot, map_resolution=12.5)
```
This constructs a map builder whose underlying map is an `AdvancedMap`.

### Behavior
- `init()` performs the initial LiDAR scan and initializes the advanced map.
- `step(m)` predicts the robot state, reads an updated LiDAR scan, runs ICP alignment, updates the `AdvancedMap`, and appends the new pose to the robot trajectory.
- `get_map()` returns the `AdvancedMap` instance, enabling access to advanced operations such as inflated occupancy and frontier detection.

### When to use
Use `AdvancedMapBuilder` when you need more than a simple occupancy map and want a map representation that supports:
- better alignment between scans and map geometry,
- probabilistic occupancy values,
- obstacle inflation for planning,
- frontier candidate extraction for exploration.

## SemanticMapBuilder
The `SemanticMapBuilder` extends `MapBuilder` to produce a `SemanticMap` that combines geometric mapping with semantic labels (rooms and objects).

### Purpose
The `SemanticMapBuilder` augments an occupancy/geometry map with semantic information derived from RGB images and point clouds. It is intended for use-cases where high-level scene understanding (room types, object labels) improves exploration, planning, or human-facing visualization.

### Key Features
- Builds a `SemanticMap` (backed by an `AdvancedMap`) that stores both geometry and semantic layers (e.g., `room`, `object`).
- Integrates LiDAR, RGB camera, and point-cloud data to localize, segment, and label the environment.
- Uses an image segmenter and a vision-language model (VLM) client to assign room labels and object semantics.

### Sensor & ML Dependencies
- LiDAR: used for geometry updates and scan-to-map alignment (via the same LiDAR-based flow as `MapBuilder`/`AdvancedMapBuilder`).
- RGB Camera: provides images for segmentation and room-label queries.
- Point Cloud: combined with image segmentation to label 3D points with semantic categories.
- Image segmentation: `physical_robot.models.segmentation.image_segmentation.ImageSegmenter` is used to produce pixel-level segments.
- Vision-Language Model (VLM): `physical_robot.models.vlm.vlm_client.VLMClient` is queried (with prompts such as `ASSIGN_ROOM_LABEL`) to produce a room-level label for the current view.

Files: see [physical_robot/map_builder/semantic_map_builder.py](physical_robot/map_builder/semantic_map_builder.py) for the implementation and references to:
- [physical_robot/maps/semantic_map.py](physical_robot/maps/semantic_map.py)
- [physical_robot/models/segmentation/image_segmentation.py](physical_robot/models/segmentation/image_segmentation.py)
- [physical_robot/models/vlm/vlm_client.py](physical_robot/models/vlm/vlm_client.py)

### Workflow (what `step(m)` does)
1. Update the internal robot state using the provided motion/odometry `m`.
2. Read an updated LiDAR scan (optionally with manual verification).
3. Capture an RGB image from the robot camera.
4. Read the point cloud snapshot.
5. Query the VLM with the RGB image and a prompt that lists known/invalid rooms to get a room label.
6. Run image segmentation on the RGB image to get pixel-level segments and labels.
7. Project/associate segmentation labels onto the point cloud and filter/format points for the semantic map.
8. Update the `SemanticMap` with both geometry (from LiDAR / point cloud) and semantics (room/object labels), and append the robot pose to the trajectory.

This workflow is implemented in `SemanticMapBuilder.step()` and tightly couples geometric updates with semantic inference so the map remains both spatially accurate and semantically meaningful.

### Visualization (`show()`)
`SemanticMapBuilder.show()` renders multiple panels using `matplotlib`:
- A geometric/occupancy view of the map.
- A semantic layer for `room` labels.
- A semantic layer for `object` labels.

### Initialization
```python
from physical_robot.robot import Robot
from physical_robot.map_builder import SemanticMapBuilder

robot = Robot()
builder = SemanticMapBuilder(robot=robot, map_resolution=10.0, manual_lidar_verification=False)
builder.init()
```

### When to use
- Use `SemanticMapBuilder` when downstream tasks (navigation, human-robot interaction, semantic exploration) require knowledge of room types or object categories beyond raw geometry.

### Notes and configuration
- The builder instantiates `ImageSegmenter` and `VLMClient` by default; these components may require model weights, credentials, or network access depending on your configured backends. Check their module-level docs for setup details.
- The `map_resolution` parameter controls the underlying `AdvancedMap` resolution; pick a resolution appropriate for your sensors and environment scale.
- For debugging or mapping quality control, enable `manual_lidar_verification` to pause for manual LiDAR checks during `step()`.

For implementation details, see the source: [physical_robot/map_builder/semantic_map_builder.py](physical_robot/map_builder/semantic_map_builder.py).
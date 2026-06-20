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

## SemanticMapBuilder
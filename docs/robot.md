# Robot Module Documentation

This document describes the `Robot` class implemented in `physical_robot/robot/robot.py`. The module acts as the main control and state abstraction for the robot platform, handling:

- robot configuration and physical dimensions
- sensor acquisition (lidar, IMU, motor telemetry)
- motion command execution for simulated, gRPC client, and physical connections
- basic motion prediction and local collision checking
- visualization helpers for robot state and paths

## File purpose

`robot.py` is the central interface between higher-level planning code and the robot hardware or simulation. It is designed around a differential-drive robot model and exposes motion control routines that can operate across multiple runtime modes:

- `simulated`: lightweight behavior for local tests and simulation workflows
- `client`: remote robot access via gRPC
- `physical`: direct interaction with the robot's hardware and Redis-published lidar data

## Class overview

### `Robot`

The class initializes the robot identity, state, geometry, controller, and communication backend.

```python
robot = Robot(connection='simulated')
```

or:

```python
robot = Robot(connection='physical')
```

#### Core state

The robot maintains a 3-element state vector:

```python
self.state = np.array([x, y, theta])
```

This represents the robot's pose in 2D space, with:

- `x`: x position in millimeters or chosen map units
- `y`: y position in millimeters or chosen map units
- `theta`: heading angle in radians

#### Geometry parameters

The constructor sets several important physical properties:

- `wheel_radius`: radius of each wheel, derived from the robot's wheel size
- `wheel_circumference`: `2 * pi * wheel_radius`
- `wheelbase_length`: distance between the left and right wheel centers
- `robot_radius`: half of the wheelbase length
- `active_distance_threshold`
- `local_planner_distance_threshold`

These values are used in motion calculations, obstacle filtering, and local planner checks.

## Initialization behavior

The `__init__` method accepts a `connection` argument and connects to the appropriate backend.

### Simulated mode

In simulated mode, the robot creates a `SimulatedLidar` instance:

```python
self.simulated_lidar = SimulatedLidar(...)
```

This is intended for map-based simulation and testing, but the sensor reading methods for simulated mode are still partially stubbed or incomplete.

### gRPC client mode

The robot configures a gRPC channel using `config['client']['channel_address']` and creates a `RobotServerStub`. This allows the robot object to call RPC methods such as:

- `GetLatestLidarData`
- `GetLatestIMUData`
- `GetMotorPositions`
- `GetMotorVelocities`
- `SendMotionCommand`

### Physical mode

The robot creates a `DynamixelController` and `RobotInterface`, then enables the configured motor profile. It also connects to Redis to read lidar data published by the physical sensor stack.

## Sensor access methods

### `read_lidar_updated(...)`

This is one of the main sensor wrappers in the class. It repeatedly calls the lower-level lidar acquisition method and validates that the data is usable before returning it.

Parameters:

- `manual_verification=False`
- `wait_for_updated_reading=False`
- `auto_retry_threshold=400`

Behavior:

- requests a fresh lidar scan
- filters points too close to the robot body
- converts raw angle values from degrees to radians
- transforms polar lidar readings into Cartesian coordinates `(x, y, z)`
- retries if the returned point count is below the threshold

Returned values are generally:

- `coords`: a point cloud in robot-local coordinates
- `lidar_data`: the filtered raw lidar array (`[angle, distance]`)
- timestamp metadata from the sensor backend

### `_get_single_lidar_reading(wait_for_updated_reading)`

This helper retrieves one lidar scan from the active backend and normalizes the values. It supports:

- simulated mode: not implemented
- client mode: gRPC sensor request
- physical mode: Redis-backed lidar stream

The method performs the following conversions:

1. reads raw `angles` and `dists`
2. strips near-range noise with a threshold around `90` units
3. flips the angle convention to a CCW-positive frame
4. converts to radians
5. computes Cartesian coordinates with `x = dist * cos(angle)` and `y = dist * sin(angle)`

### `read_imu_data()`

This retrieves IMU measurements from the client backend and returns:

```python
accel_data = [accel_x, accel_y, accel_z]
gyro_data = [gyro_x, gyro_y, gyro_z]
```

Physical mode is currently unimplemented.

### `read_motor_positions()`

Fetches the current left and right motor positions from the client backend and returns a NumPy array shaped like:

```python
np.array([left, right])
```

### `read_motor_velocities()`

Fetches instantaneous motor velocities from the client backend in the same left/right form.

## Motion modeling and prediction

The class implements differential-drive motion logic and predicts the robot's next pose from wheel motion.

### `motion_command_to_pseudo_motor_diffs(motion_command)`

Converts a command such as:

```python
['linear', 100.0]
```

or

```python
['angular', np.pi / 2]
```

into pseudo motor differential pairs. This is a conversion layer between high-level motion semantics and wheel effort.

Examples in the file include:

- linear motion: `[pseudo_avg_motion, -pseudo_avg_motion]`
- angular motion: `[pseudo_avg_motion, pseudo_avg_motion]`

### `predict_state(state, motor_position_differential)`

Given a current robot state and a measured or commanded wheel differential, this method infers the resulting pose.

It classifies the motion into one of the following:

- left in-place rotation
- right in-place rotation
- forward translation
- backward translation

The pose update takes the form of:

- linear motion: `state + [cos(theta), sin(theta), 0] * avg_motion`
- angular motion: `state + [0, 0, rotation_delta]`

### `command_motion_and_predict_state(state, motion_command)`

This method wraps the execution flow:

1. send the motion command
2. compute the resulting motor differential
3. predict the resulting state
4. return both values

## Motion execution

### `move_linear(mm=100)`

This method moves the robot in a straight line by computing the required wheel pulses for the requested distance.

It performs the following steps:

1. reads the initial motor positions
2. computes wheel travel distance from the requested millimeter distance
3. converts that to required revolutions and then pulses
4. commands the target motor positions
5. optionally waits for the motion to finish
6. applies a safety guard if `guard_active_motion` is enabled
7. returns the computed linear displacement from the motor deltas

### `move_angular(rad=np.pi/2)`

This method rotates the robot in place by a commanded angle in radians, using the wheelbase geometry to compute the required wheel travel offset.

### `compute_linear_motion(init_mp, final_mp)`

Computes the net linear distance produced by the motor position change. It normalizes wrap-around motor encoder values and converts pulse deltas into revolutions and then linear travel distance.

### `compute_rotation_motion(init_mp, final_mp)`

Computes the net angular rotation from the motor position differential. This uses the wheel radius and wheelbase length to convert encoder motion into pose rotation.

### `command_motion_trial(motion_command)`

This is the top-level execution method for robot motion. It routes the command based on the active connection type:

- `simulated`: uses pseudo wheel diffs
- `client`: sends the request to the gRPC server
- `physical`: calls `move_linear` or `move_angular`

## Local planning and safety monitoring

### `local_planner(motion_command)`

This method attempts to reduce a motion command when a future path would collide with nearby lidar readings. It:

- reads the current lidar scan
- filters points too near the robot
- creates a line segment representing the commanded travel path
- measures distance from obstacle points to that segment
- finds the nearest feasible stopping point if a collision is imminent
- returns a shortened motion command instead of the original one

This is a simple local collision-avoidance helper and is intended to be used before committing to motion.

### `active_lidar_monitoring(pause_motion, thread_stop)`

A background thread monitors lidar input while motion is active. If the minimum distance falls below `active_distance_threshold`, it sets a pause event to stop or slow down motion.

### `advanced_active_lidar_monitoring(...)`

This is a more advanced motion guard that compares the remaining target distance against the current lidar state while the robot is moving. It is meant to pause movement if an obstacle is encountered before the robot reaches its target pose.

## Visualization and path utilities

### `draw_state(ax, state)`

This helper renders a robot footprint, wheel locations, and heading direction on a matplotlib axis.

It creates:

- a blue robot body
- two black wheels rotated to match the robot heading
- a red heading vector indicating orientation

### `path_to_motion_commands(path)`

Delegates path conversion to `RobotController.compute_motion_commands(path)`.

### `terminate()`

A stub for cleanup logic. Currently not implemented.

## Example usage

```python
from physical_robot.robot.robot import Robot
import numpy as np

robot = Robot(connection='simulated')

# Move forward 400 mm
motion = ['linear', 400]
robot.command_motion_trial(motion)

# Rotate 90 degrees counter-clockwise (depending on conventions)
robot.command_motion_trial(['angular', np.pi / 2])
```

## Important caveats and current limitations

This file is a working control layer, but several areas are intentionally incomplete or partially implemented:

- `simulated` lidar and some sensor read methods are stubbed
- physical-mode motor read methods raise `NotImplementedError`
- some TODO comments indicate cleanup and refactoring is still needed
- the geometry and sign conventions for some motion directions are sensitive and may require calibration
- motion safety logic is experimental and relies on thread coordination and lidar thresholding

## Relationship to the rest of the project

`Robot` is the high-level controller used by the broader robot system. It sits between:

- low-level hardware drivers (`DynamixelController`, `RobotInterface`)
- map and planning components (`RobotController` and related path logic)
- sensor pipelines (`lidar`, `IMU`, `Redis`, gRPC)
- visualization and debugging utilities

In practice, this file is the main API object used to command motion and gather robot state while the system is running.

## Summary

The `Robot` class is the main abstraction for controlling and observing the robot. It combines pose tracking, sensor access, motor control, planner hooks, and visualization into one central interface. It is a foundational module for movement, safety checks, and higher-level navigation code in the project.

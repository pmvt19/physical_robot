# Monitoring Suite Documentation

This document describes the visualization and live-monitoring utilities in `physical_robot/monitoring/`. These scripts are intended to stream robot sensor output to local visualization tools so operators can inspect the robot state while it is running.

The monitoring suite includes:

- `viz_camera.py` for camera display
- `viz_imu.py` for IMU plotting via Rerun
- `viz_point_cloud.py` for 3D point-cloud visualization
- `viz_robot_lidar.py` for lidar-based spatial visualization

The `sensor_health_checks.py` file is intentionally excluded from this documentation at your request because it has not yet been validated or tested.

## Purpose of the monitoring suite

The monitoring tools are used to observe the robot in real time during development and testing. They help answer questions such as:

- Is the lidar returning valid distance data?
- Is the camera stream updating correctly?
- Are the accelerometer and gyroscope values reasonable?
- Does the environment point cloud look coherent?

The scripts are development-facing utilities rather than production monitoring systems.

## Common architecture

Most of the monitoring scripts use the same pattern:

1. create a `Robot` client object
2. request live data from the robot backend
3. format or visualize the output in a local UI
4. loop until interrupted or the time limit expires

The monitoring layer relies on the robot client API from `physical_robot.robot` and uses either:

- OpenCV for image windows
- Rerun for 3D/scalar visualization

## `viz_camera.py`

### File purpose

This script streams the RGB and depth camera outputs and displays them in OpenCV windows.

### Behavior

The file creates a `Robot(connection="client")` instance and then continuously calls:

```python
rgb_numpy, depth_numpy = robot.read_rgb_camera()
```

It then shows the images using:

```python
cv2.imshow("frame", rgb_numpy)
cv2.imshow("depth frame", depth_numpy)
```

The loop exits when the user presses `q`.

### Dependencies

- OpenCV (`cv2`)
- `physical_robot.robot.Robot`

### Typical use

This is useful for validating the camera feed, checking image quality, and confirming the robot is returning color and depth frames from the hardware or backend server.

## `viz_imu.py`

### File purpose

This script visualizes accelerometer and gyroscope readings using Rerun.

### Behavior

The script initializes a Rerun session:

```python
rr.init("IMU Data", spawn=True, init_logging=True)
```

Then it repeatedly reads IMU data from the robot client:

```python
accel_data, gyro_data = robot.read_imu()
```

Once the data is read, it logs scalar values for each axis:

```python
rr.log("accelerometer x", rr.Scalars(float(accel_data[0])))
rr.log("accelerometer y", rr.Scalars(accel_data[1]))
rr.log("accelerometer z", rr.Scalars(accel_data[2]))

rr.log("gyroscope x", rr.Scalars(gyro_data[0]))
rr.log("gyroscope y", rr.Scalars(gyro_data[1]))
rr.log("gyroscope z", rr.Scalars(gyro_data[2]))
```

It also logs time using:

```python
rr.set_time("time", duration=time.time() - start_time)
```

### Dependencies

- `numpy`
- `rerun`
- `physical_robot.robot.Robot`

### Typical use

This script is helpful for checking whether the robot is moving or rotating as expected, and for observing IMU stability over time.

## `viz_point_cloud.py`

### File purpose

This script visualizes a 3D point cloud from the robot's sensor output.

### Behavior

The script creates a Rerun session and then repeatedly reads a point-cloud representation from the robot:

```python
coords, colors = robot.read_point_cloud()
```

It then reshapes the coordinate array and logs it as a 3D point cloud:

```python
coords = np.stack((coords[:, 0], coords[:, 1], coords[:, 2]), axis=1)
rr.log("points", rr.Points3D(coords, colors=colors))
```

This gives a live 3D view of the environment as observed from the robot sensor stack.

### Dependencies

- `numpy`
- `rerun`
- `physical_robot.robot.Robot`

### Typical use

This is useful for checking scene geometry, obstacle consistency, and the quality of point-cloud generation in a spatial context.

## `viz_robot_lidar.py`

### File purpose

This script visualizes lidar data in a spatial 3D view using Rerun.

### Behavior

The script creates a `Robot` instance and then repeatedly calls:

```python
coords, raw_lidar_data = robot.read_lidar_updated(wait_for_updated_reading=True)
```

It then logs the filtered coordinates as a `Points3D` entity:

```python
rr.log("points", rr.Points3D(coords))
```

A second point is also logged at the origin for reference:

```python
rr.log(
    "points v2",
    rr.Points3D([[[0.0, 0.0, 0.0]]], colors=[0, 255, 0], radii=10.0),
)
```

This provides a quick visual reference for the robot center and the lidar point cloud relative to it.

### Dependencies

- `numpy`
- `rerun`
- `physical_robot.robot.Robot`

### Typical use

The script is intended for live lidar debugging: checking whether the sensor is reading obstacles correctly, whether a scan is being filtered as expected, and whether the robot frame is consistent with the environment.

## Execution notes

Most of these scripts are intended to be run directly as examples or live debugging tools, for example:

```bash
python physical_robot/monitoring/viz_camera.py
python physical_robot/monitoring/viz_imu.py
python physical_robot/monitoring/viz_point_cloud.py
python physical_robot/monitoring/viz_robot_lidar.py
```

These utilities assume the robot is reachable through the configured `Robot(connection="client")` workflow or that a matching backend is already running.

## Current limitations

This monitoring layer is useful for development, but it is not a complete production monitoring stack.

Current limitations include:

- scripts are mostly standalone entrypoints rather than reusable library components
- real-time behavior depends on the robot backend being available and healthy
- some visualization code is exploratory and not yet standardized across all sensors
- health-check logic is intentionally omitted from this document because it has not been tested

## Summary

The monitoring suite provides a lightweight live-debugging layer for the robot system. It gives developers immediate feedback from camera streams, IMU data, point clouds, and lidar scans without needing to build a full dashboard. These tools are most valuable during development, calibration, and troubleshooting sessions.

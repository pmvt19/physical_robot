# gRPC Protocol Documentation

This document explains the contract defined in `physical_robot/protos/robot_data.proto`. The file defines the communication schema used by the robot's gRPC server-client architecture, including sensor payloads, motion commands, and robot state messages.

## Why this file matters

The protocol file is the API contract between:

- the robot runtime or hardware layer
- the gRPC server
- the client application that requests sensor data or sends commands

Without these message definitions, the Python-generated stubs would not know how to serialize or deserialize request/response payloads correctly.

## Package and message model

The proto file is scoped under the package:

```proto
package physical_robot;
```

This keeps the generated code organized and makes the message types explicit to the project.

## Core message types

### `LidarData`

```proto
message LidarData {
    repeated float angles = 1;
    repeated float dists = 2;
    int64 timestamp = 3;
}
```

Represents a complete lidar scan.

Fields:

- `angles`: angle values for each lidar sample
- `dists`: corresponding distance values for each angle
- `timestamp`: time marker associated with the scan

This is the main payload returned by the lidar sensor stream.

### `Acknowledge`

```proto
message Acknowledge {
    bool success = 1;
    string message = 2;
}
```

Simple acknowledgement message used as a request wrapper for sensor queries. It tells the server that the client is ready to receive data.

### `MotionCommand`

```proto
message MotionCommand {
    string motion_type = 1;
    float distance = 2;
}
```

Used to ask the robot to perform a motion.

Fields:

- `motion_type`: high-level command type, such as `linear` or `angular`
- `distance`: commanded distance or angular amount, depending on the motion type

### `MotionDistance`

```proto
message MotionDistance {
    float left_wheel_dist = 1;
    float right_wheel_dist = 2;
}
```

Returned after a motion request. It records how far each wheel traveled as part of the commanded motion.

### `NumpyArray`

```proto
message NumpyArray {
    bytes img_bytes = 1;
    repeated int64 shape = 2;
    string type = 3;
}
```

A serialized NumPy-style array container used for image or point-cloud payloads.

Fields:

- `img_bytes`: raw byte payload
- `shape`: array dimension shape
- `type`: value type string

This is used as a general container for large arrays that need to cross the gRPC boundary.

### `CameraData`

```proto
message CameraData {
    NumpyArray rgb_img = 1;
    NumpyArray depth_img = 2;
    int64 timestamp = 3;
}
```

Contains synchronized RGB and depth image data from the camera stack.

### `PointCloudData`

```proto
message PointCloudData {
    NumpyArray point_cloud_coords = 1;
    NumpyArray point_cloud_colors = 2;
    int64 timestamp = 3;
}
```

Provides 3D point-cloud data, typically from the OAK-D or similar depth camera pipeline.

### `IMUData`

```proto
message IMUData {
    float accel_x = 1;
    float accel_y = 2;
    float accel_z = 3;

    float gyro_x = 4;
    float gyro_y = 5;
    float gyro_z = 6;
}
```

Carries accelerometer and gyroscope measurements.

### `OakdLiteData`

```proto
message OakdLiteData {
    CameraData camera_data = 1;
    IMUData imu_Data = 2;
    PointCloudData point_cloud_data = 3;
    int64 timestamp = 4;
}
```

A composite message that groups all relevant OAK-D Lite sensor outputs into a single payload.

### `RobotMotorPositions`

```proto
message RobotMotorPositions {
    int64 motor_position_left = 1;
    int64 motor_position_right = 2;
}
```

Stores the current encoder positions for the left and right drive motors.

### `RobotMotorVelocities`

```proto
message RobotMotorVelocities {
    int64 motor_velocity_left = 3;
    int64 motor_velocity_right = 4;
}
```

Stores the current motor velocities.

### `RobotState`

```proto
message RobotState {
    RobotMotorPositions positions = 1;
    RobotMotorVelocities velocities = 2;
    NumpyArray robot_state = 3;
}
```

Aggregates robot state information, including motor telemetry and a generic robot state array.

### `MonitoringState`

```proto
message MonitoringState {
    LidarData lidar_data = 1;
    CameraData camera_data = 2;
    IMUData imu_data = 3;
    RobotState robot_state = 4;
}
```

A combined monitoring payload that packages the primary sensor and state information together.

## RPC service definitions

The file defines a `RobotServer` gRPC service.

```proto
service RobotServer {
    rpc GetLatestLidarData(Acknowledge) returns (LidarData);

    rpc GetLatestImageData(Acknowledge) returns (CameraData);

    rpc GetLatestIMUData(Acknowledge) returns (IMUData);

    rpc GetLatestOakdLiteData(Acknowledge) returns (OakdLiteData);

    rpc SendMotionCommand(MotionCommand) returns (MotionDistance);

    rpc GetMotorPositions(Acknowledge) returns (RobotMotorPositions);

    rpc GetMotorVelocities(Acknowledge) returns (RobotMotorVelocities);
}
```

### `GetLatestLidarData`

Returns the most recent lidar scan.

### `GetLatestImageData`

Returns current camera image data.

### `GetLatestIMUData`

Returns the most recent IMU reading.

### `GetLatestOakdLiteData`

Returns combined camera, IMU, and point-cloud data for the OAK-D Lite sensor stack.

### `SendMotionCommand`

Sends a motion request and receives the resulting wheel-distance response.

### `GetMotorPositions`

Returns the current left/right motor encoder positions.

### `GetMotorVelocities`

Returns the current left/right motor velocities.

## Design notes

A few useful observations about this protocol:

- the service is centered around sensor polling and motion execution
- many calls use `Acknowledge` as an empty request wrapper to keep the API simple
- `NumpyArray` is used to move large array-like data across the boundary
- composite payloads such as `OakdLiteData` reduce the number of individual RPC calls needed for a full sensor snapshot

## Current caveats

There are a few naming and consistency details worth noting:

- `imu_Data` in `OakdLiteData` uses a capital `D` in the field name, which is slightly inconsistent with protobuf naming conventions
- `MonitoringState` is defined but not clearly used by the rest of the runtime in the current codebase
- the proto contract is broad enough for development, but some message types appear to be more experimental than fully finalized

## Summary

The `.proto` file is an important part of the project because it defines the exact data contracts used across the robot system. It is the backbone of the client/server communication layer and should be documented alongside the robot controller and hardware interfaces so future contributors understand how data moves between the robot, the server, and the client application.

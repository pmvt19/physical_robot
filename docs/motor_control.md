# Motor Control Documentation

This document describes the low-level hardware control layer used by the robot's drivetrain. It covers the two closely related modules:

- `physical_robot/hardware/motors/dxl_controller.py`
- `physical_robot/hardware/motors/robot_motor_interface.py`

Together, these files provide the interface between the robot application and the Dynamixel motors.

## Overview

The system uses the Dynamixel SDK to communicate with two motors over a serial port. The design is intentionally split into two layers:

1. `DynamixelController` handles low-level register access and packet communication.
2. `RobotInterface` wraps those calls in a higher-level robot API, adding locking, motion commands, and drivetrain semantics.

This keeps the code modular: the controller knows how to talk to the motors, while the robot interface knows how the robot is supposed to move.

## Dynamixel controller

### File purpose

`dxl_controller.py` is the direct Dynamixel SDK wrapper for the XL-type motors. It provides methods to:

- open the serial port and configure the baud rate
- set motor operating mode
- read and write velocity and position registers
- enable or disable torque
- check communication success and error states

### Key constants

The file defines the standard Dynamixel register addresses and operating modes used by the motors.

```python
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_PROF_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
```

Important mode values:

- `TORQUE_ENABLE = 1`
- `TORQUE_DISABLE = 0`
- `VELOCITY_CONTROL_MODE = 1`
- `POSITION_CONTROL_MODE = 3`
- `EXTENDED_POSITION_CONTROL_MODE = 4`
- `PWM_CONTROL_MODE = 16`

The communication protocol is configured as:

```python
PROTOCOL_VERSION = 2.0
BAUDRATE = 57600
```

### `DynamixelController`

The class initializes the SDK port and packet handlers and stores motor IDs for each connected device.

```python
controller = DynamixelController(device_name="/dev/ttyUSB0", motor_ids=[1, 2])
```

Initialization steps:

1. create `PortHandler(device_name)`
2. create `PacketHandler(PROTOCOL_VERSION)`
3. open the serial port
4. set the baud rate
5. retain motor IDs and state metadata

The class also defines `max_motor_position = 2**32`, which is used for wraparound-safe position calculations when working with the motor's 32-bit encoder values.

### Communication and error handling

The `check_ok` method validates the return status from Dynamixel SDK communication. It checks both:

- `dxl_comm_result`
- `dxl_error`

If communication fails, it logs a clear error via the configured logger. This makes it easier to debug serial or command failures without crashing the process.

### Motor configuration methods

#### `set_operating_mode(id, mode=VELOCITY_CONTROL_MODE)`

Sets the operating mode for a specific motor using the Dynamixel register for operating mode.

This is used to switch the motor between:

- velocity control
- position control
- extended position control

#### `set_profile_velocity(id, velocity)`

Configures the motor's profile velocity register. This is useful for motion tuning and smoothing when accelerating to a target setpoint.

#### `set_torque(id, value=TORQUE_DISABLE)`

Enables or disables the motor's torque output.

### Position and velocity methods

#### `set_position(id, position)`

Writes the requested goal position to the motor.

#### `set_velocity(id, velocity_rpm)`

Converts the desired RPM into Dynamixel units and writes the goal velocity register.

The conversion uses the approximation:

```python
goal_velocity_unit = int(velocity_rpm / 0.229)
```

#### `get_velocity(id)`

Reads the current motor velocity from the present velocity register.

#### `get_position(id)`

Reads the current encoder position from the present position register.

### Example usage

```python
controller = DynamixelController(device_name="/dev/ttyUSB0", motor_ids=[1, 2])

controller.set_torque(1, TORQUE_DISABLE)
controller.set_torque(2, TORQUE_DISABLE)

controller.set_operating_mode(1, VELOCITY_CONTROL_MODE)
controller.set_operating_mode(2, VELOCITY_CONTROL_MODE)

controller.set_torque(1, TORQUE_ENABLE)
controller.set_torque(2, TORQUE_ENABLE)

print(controller.get_position(1))
```

This layer is intentionally low-level and does not encode robot-specific behavior. It simply exposes the direct Dynamixel operations needed by the rest of the system.

## Robot motor interface

### File purpose

`robot_motor_interface.py` wraps the low-level `DynamixelController` into a robot-centric API for a differential-drive chassis. It provides methods for:

- setting torque and mode for both motors
- reading motor velocities and individual positions
- issuing drivetrain commands
- stopping motion
- managing motion-safe locking

### Design principles

The class is built around a differential-drive robot model, where the left and right wheels are treated as a pair. The code assumes:

- motor 1 and motor 2 correspond to the robot's two driven wheels
- `self.linear_velocity` is the commanded wheel speed in RPM
- the robot chassis uses a left/right differential drive model

### Locking and thread safety

The interface stores a `Lock` in `self.controller_lock` and uses it around all motor operations. This helps prevent simultaneous writes from competing threads while motion commands are being issued or while the robot is being monitored.

The helper classes `MockLock` and `LoggedLock` exist as lightweight lock abstractions for testing or logging use, but `RobotInterface` currently uses the standard `threading.Lock` by default.

### `RobotInterface`

```python
controller = DynamixelController(device_name="/dev/ttyUSB0", motor_ids=[1, 2])
ri = RobotInterface(controller=controller)
```

In `__init__`, the interface:

- stores the `DynamixelController`
- initializes `self.linear_velocity = 10`
- creates a lock
- disables torque
- sets the motor mode to `EXTENDED_POSITION_CONTROL_MODE`
- re-enables torque
- sets up geometry constants
- defines pulse-per-revolution metadata

Important configuration values:

```python
self.r = 66.5 / 2
self.L = 210
self.pulse_per_rev = 4096
```

These values are used by the higher-level robot control code when computing linear and rotational motion from motor encoder deltas.

### Core control methods

#### `set_torque(torque)`

Applies the same torque state to both motors.

```python
ri.set_torque(TORQUE_ENABLE)
ri.set_torque(TORQUE_DISABLE)
```

#### `set_mode(mode)`

Sets the operating mode for both motors at once.

This is convenient because the robot typically runs both motors in the same mode.

#### `set_profile_velocity(velocity=100)`

Configures each motor's profile velocity.

#### `get_motor_velocity()`

Reads both motors' present velocities and returns them as a NumPy array:

```python
np.array([left_velocity, right_velocity])
```

#### `set_motor_positions(positions)`

Commands both motors to a target set of positions:

```python
ri.set_motor_positions([desired_left_pos, desired_right_pos])
```

#### `get_motor_positions()`

Reads both present positions and returns them in a NumPy array of dtype `np.int64`.

### Drive commands

The class exposes a small differential-drive command set that maps directly to robot motion actions:

#### `move_forward()`

Sets the motors to move in opposite directions to translate the robot forward.

```python
self.controller.set_velocity(id=1, velocity_rpm=self.linear_velocity)
self.controller.set_velocity(id=2, velocity_rpm=-self.linear_velocity)
```

#### `move_backward()`

Reverses the wheel directions for backward translation.

#### `rotate_right()`

Commands both motors to rotate in the same direction to spin the robot to the right.

#### `rotate_left()`

Commands both motors in the opposite direction for left rotation.

#### `stop_motion()`

Sets both motor velocities to zero.

### Example usage

```python
controller = DynamixelController(device_name="/dev/ttyUSB0", motor_ids=[1, 2])
ri = RobotInterface(controller=controller)

ri.move_forward()
# ... wait for motion
ri.stop_motion()

ri.rotate_left()
# ... wait for rotation
ri.stop_motion()
```

## How the two layers work together

The relationship between the two files is straightforward:

- `DynamixelController` talks to the hardware registers.
- `RobotInterface` groups those operations into robot-specific movement primitives.

For example, the higher-level robot code can call:

```python
ri.set_motor_positions([goal_left, goal_right])
```

and the interface converts that into two low-level calls on the controller:

```python
self.controller.set_position(id=1, position=positions[0])
self.controller.set_position(id=2, position=positions[1])
```

This allows the rest of the project to reason about robot-level motion instead of raw motor register values.

## Important caveats

These modules are functional but still reflect an active development stage:

- some methods are intentionally minimal or incomplete
- several helper methods remain stubs (`get_motor_name_from_id`, `get_motor_id_from_name`, `get_operating_mode`)
- the code assumes a fixed motor numbering convention for the robot
- some velocity and rotation sign conventions may require calibration on the physical platform
- the `move_forward` and rotation methods are simple controller-level commands and may not fully account for all physical tuning factors

## Summary

`dxl_controller.py` is the low-level hardware driver for Dynamixel motors, while `robot_motor_interface.py` provides the robot-facing abstraction used to command the drive system. Together they are the foundation of the robot's motor control layer and connect the application logic to the actual hardware.

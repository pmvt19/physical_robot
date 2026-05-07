from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Iterable as _Iterable, Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class LidarData(_message.Message):
    __slots__ = ("angles", "dists", "timestamp")
    ANGLES_FIELD_NUMBER: _ClassVar[int]
    DISTS_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    angles: _containers.RepeatedScalarFieldContainer[float]
    dists: _containers.RepeatedScalarFieldContainer[float]
    timestamp: int
    def __init__(self, angles: _Optional[_Iterable[float]] = ..., dists: _Optional[_Iterable[float]] = ..., timestamp: _Optional[int] = ...) -> None: ...

class Acknowledge(_message.Message):
    __slots__ = ("success", "message")
    SUCCESS_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    success: bool
    message: str
    def __init__(self, success: bool = ..., message: _Optional[str] = ...) -> None: ...

class MotionCommand(_message.Message):
    __slots__ = ("motion_type", "distance")
    MOTION_TYPE_FIELD_NUMBER: _ClassVar[int]
    DISTANCE_FIELD_NUMBER: _ClassVar[int]
    motion_type: str
    distance: float
    def __init__(self, motion_type: _Optional[str] = ..., distance: _Optional[float] = ...) -> None: ...

class MotionDistance(_message.Message):
    __slots__ = ("left_wheel_dist", "right_wheel_dist")
    LEFT_WHEEL_DIST_FIELD_NUMBER: _ClassVar[int]
    RIGHT_WHEEL_DIST_FIELD_NUMBER: _ClassVar[int]
    left_wheel_dist: float
    right_wheel_dist: float
    def __init__(self, left_wheel_dist: _Optional[float] = ..., right_wheel_dist: _Optional[float] = ...) -> None: ...

class NumpyArray(_message.Message):
    __slots__ = ("img_bytes", "shape", "type")
    IMG_BYTES_FIELD_NUMBER: _ClassVar[int]
    SHAPE_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    img_bytes: bytes
    shape: _containers.RepeatedScalarFieldContainer[int]
    type: str
    def __init__(self, img_bytes: _Optional[bytes] = ..., shape: _Optional[_Iterable[int]] = ..., type: _Optional[str] = ...) -> None: ...

class CameraData(_message.Message):
    __slots__ = ("rgb_img", "depth_img", "timestamp")
    RGB_IMG_FIELD_NUMBER: _ClassVar[int]
    DEPTH_IMG_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    rgb_img: NumpyArray
    depth_img: NumpyArray
    timestamp: int
    def __init__(self, rgb_img: _Optional[_Union[NumpyArray, _Mapping]] = ..., depth_img: _Optional[_Union[NumpyArray, _Mapping]] = ..., timestamp: _Optional[int] = ...) -> None: ...

class PointCloudData(_message.Message):
    __slots__ = ("point_cloud_coords", "point_cloud_colors", "timestamp")
    POINT_CLOUD_COORDS_FIELD_NUMBER: _ClassVar[int]
    POINT_CLOUD_COLORS_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    point_cloud_coords: NumpyArray
    point_cloud_colors: NumpyArray
    timestamp: int
    def __init__(self, point_cloud_coords: _Optional[_Union[NumpyArray, _Mapping]] = ..., point_cloud_colors: _Optional[_Union[NumpyArray, _Mapping]] = ..., timestamp: _Optional[int] = ...) -> None: ...

class IMUData(_message.Message):
    __slots__ = ("accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z")
    ACCEL_X_FIELD_NUMBER: _ClassVar[int]
    ACCEL_Y_FIELD_NUMBER: _ClassVar[int]
    ACCEL_Z_FIELD_NUMBER: _ClassVar[int]
    GYRO_X_FIELD_NUMBER: _ClassVar[int]
    GYRO_Y_FIELD_NUMBER: _ClassVar[int]
    GYRO_Z_FIELD_NUMBER: _ClassVar[int]
    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    def __init__(self, accel_x: _Optional[float] = ..., accel_y: _Optional[float] = ..., accel_z: _Optional[float] = ..., gyro_x: _Optional[float] = ..., gyro_y: _Optional[float] = ..., gyro_z: _Optional[float] = ...) -> None: ...

class OakdLiteData(_message.Message):
    __slots__ = ("camera_data", "imu_Data", "point_cloud_data", "timestamp")
    CAMERA_DATA_FIELD_NUMBER: _ClassVar[int]
    IMU_DATA_FIELD_NUMBER: _ClassVar[int]
    POINT_CLOUD_DATA_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    camera_data: CameraData
    imu_Data: IMUData
    point_cloud_data: PointCloudData
    timestamp: int
    def __init__(self, camera_data: _Optional[_Union[CameraData, _Mapping]] = ..., imu_Data: _Optional[_Union[IMUData, _Mapping]] = ..., point_cloud_data: _Optional[_Union[PointCloudData, _Mapping]] = ..., timestamp: _Optional[int] = ...) -> None: ...

class RobotMotorPositions(_message.Message):
    __slots__ = ("motor_position_left", "motor_position_right")
    MOTOR_POSITION_LEFT_FIELD_NUMBER: _ClassVar[int]
    MOTOR_POSITION_RIGHT_FIELD_NUMBER: _ClassVar[int]
    motor_position_left: int
    motor_position_right: int
    def __init__(self, motor_position_left: _Optional[int] = ..., motor_position_right: _Optional[int] = ...) -> None: ...

class RobotMotorVelocities(_message.Message):
    __slots__ = ("motor_velocity_left", "motor_velocity_right")
    MOTOR_VELOCITY_LEFT_FIELD_NUMBER: _ClassVar[int]
    MOTOR_VELOCITY_RIGHT_FIELD_NUMBER: _ClassVar[int]
    motor_velocity_left: int
    motor_velocity_right: int
    def __init__(self, motor_velocity_left: _Optional[int] = ..., motor_velocity_right: _Optional[int] = ...) -> None: ...

class RobotState(_message.Message):
    __slots__ = ("positions", "velocities", "robot_state")
    POSITIONS_FIELD_NUMBER: _ClassVar[int]
    VELOCITIES_FIELD_NUMBER: _ClassVar[int]
    ROBOT_STATE_FIELD_NUMBER: _ClassVar[int]
    positions: RobotMotorPositions
    velocities: RobotMotorVelocities
    robot_state: NumpyArray
    def __init__(self, positions: _Optional[_Union[RobotMotorPositions, _Mapping]] = ..., velocities: _Optional[_Union[RobotMotorVelocities, _Mapping]] = ..., robot_state: _Optional[_Union[NumpyArray, _Mapping]] = ...) -> None: ...

class MonitoringState(_message.Message):
    __slots__ = ("lidar_data", "camera_data", "imu_data", "robot_state")
    LIDAR_DATA_FIELD_NUMBER: _ClassVar[int]
    CAMERA_DATA_FIELD_NUMBER: _ClassVar[int]
    IMU_DATA_FIELD_NUMBER: _ClassVar[int]
    ROBOT_STATE_FIELD_NUMBER: _ClassVar[int]
    lidar_data: LidarData
    camera_data: CameraData
    imu_data: IMUData
    robot_state: RobotState
    def __init__(self, lidar_data: _Optional[_Union[LidarData, _Mapping]] = ..., camera_data: _Optional[_Union[CameraData, _Mapping]] = ..., imu_data: _Optional[_Union[IMUData, _Mapping]] = ..., robot_state: _Optional[_Union[RobotState, _Mapping]] = ...) -> None: ...

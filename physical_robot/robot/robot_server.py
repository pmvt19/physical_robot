# For GetLatestLidarData 

# 1. Run Redis Lidar Publisher
# 2. Read Latest Lidar Redis Info and Publish it if GetLatestLidarData is called

# For SendMotionCommand
# 1. Robot Will Use RobotInterface to move the robot according to the command
# 2. Robot will send the distance of its wheel motion back to the laptop

from concurrent import futures

import time
import grpc
import physical_robot.generated.robot_data_pb2 as pb2
import physical_robot.generated.robot_data_pb2_grpc as pb2_grpc
import numpy as np
import redis
from physical_robot.robot import Robot

from physical_robot.utils import register_logger
import logging

logger = register_logger(logger_name=__name__, log_filename='robot_server', level=logging.DEBUG, std_err=False)

class RobotServerServicer(pb2_grpc.RobotServer):
    """The server implementation of the RobotServer service."""
    def __init__(self):
        self.robot = Robot(connection='physical')
        # self.redis_client = redis.Redis(host='localhost', port=6379, db=0)

    def _getAndFormatTimestamp(self, key):
        return int(float(self.robot.redis_client.get(key)) * 100000) # TODO: THIS IS BAD


    def GetLatestLidarData(self, request, context):
        """Implements the RPC method."""
        print(f"Server received request: success={request.success}, message='{request.message}'")

        # TODO: Not Required, Request is not an input that matters
        if not request.success:
            # You can set an error status if the request indicates failure
            context.set_code(grpc.StatusCode.INVALID_ARGUMENT)
            context.set_details(request.message)
            return pb2.LidarData() # Return an empty response
        
        lidar_data = np.frombuffer(self.robot.redis_client.get("lidar_data")).reshape(-1, 2)
        timestamp = int(float(self.robot.redis_client.get('time')) * 100000) # TODO: THIS IS BAD

        angles = lidar_data[:, 0]
        dists = lidar_data[:, 1]

        # 1. Create the data to send back
        fake_data = pb2.LidarData(
            angles=angles,
            dists=dists,
            timestamp=timestamp
        )

        # 2. Return the data
        return fake_data
    
    def GetLatestImageData(self, request, context):
        """Implements the RPC method."""

        # Get All RGB Image Data
        rgb_img_bytes = self.robot.redis_client.get('rgb_img')
        rgb_img_shape = self.robot.redis_client.get('rgb_img_shape')
        rgb_img_type = self.robot.redis_client.get('rgb_img_type')

        # Get All Depth Image Data
        depth_img_bytes = self.robot.redis_client.get('stereo_img')
        depth_img_shape = self.robot.redis_client.get('stereo_img_shape')
        depth_img_type = self.robot.redis_client.get('stereo_img_type')

        # Publish Time Images Were Grabbed
        timestamp = self._getAndFormatTimestamp(key='camera_time')

        ## TODO: DO SOME PROCESSING TO THE RAW DATA??

        ## TODO: DO SOME PROCESSING TO THE RAW DATA??

        rgb_img_shape = tuple(np.frombuffer(rgb_img_shape, dtype=np.int64))
        depth_img_shape = tuple(np.frombuffer(depth_img_shape, dtype=np.int64))

        rgb_img = pb2.NumpyArray(
            img_bytes=rgb_img_bytes,
            shape=rgb_img_shape,
            type=rgb_img_type
        )

        depth_img = pb2.NumpyArray(
            img_bytes=depth_img_bytes,
            shape=depth_img_shape,
            type=depth_img_type
        )

        camera_data = pb2.CameraData(
            rgb_img=rgb_img,
            depth_img=depth_img,
            timestamp=timestamp
        )

        return camera_data
    
    def GetLatestIMUData(self, request, context):
        # Publish Accelerometer Data
        accel_x = self.robot.redis_client.get('accel_x')
        accel_y = self.robot.redis_client.get('accel_y')
        accel_z = self.robot.redis_client.get('accel_z')

        # Publish Gyroscope Data
        gyro_x = self.robot.redis_client.get('gyro_x')
        gyro_y = self.robot.redis_client.get('gyro_y')
        gyro_z = self.robot.redis_client.get('gyro_z')

        imu_data = pb2.IMUData(
            accel_x=float(accel_x),
            accel_y=float(accel_y),
            accel_z=float(accel_z),
            gyro_x=float(gyro_x),
            gyro_y=float(gyro_y),
            gyro_z=float(gyro_z)
        )

        return imu_data
    
    def GetLatestPointCloudData(self, request, context):
        # Get Point Cloud Data
        pcl_coords_bytes = self.robot.redis_client.get('pcl_coords')
        pcl_coords_shape = self.robot.redis_client.get('pcl_coords_shape')
        pcl_coords_type = self.robot.redis_client.get('pcl_coords_type')

        pcl_colors_bytes = self.robot.redis_client.get('pcl_colors')
        pcl_colors_shape = self.robot.redis_client.get('pcl_colors_shape')
        pcl_colors_type = self.robot.redis_client.get('pcl_colors_type')

        pcl_coords_shape = tuple(np.frombuffer(pcl_coords_shape, dtype=np.int64))
        pcl_colors_shape = tuple(np.frombuffer(pcl_colors_shape, dtype=np.int64))

        pcl_coords = pb2.NumpyArray(
            img_bytes=pcl_coords_bytes,
            shape=pcl_coords_shape,
            type=pcl_coords_type
        )

        pcl_colors = pb2.NumpyArray(
            img_bytes=pcl_colors_bytes,
            shape=pcl_colors_shape,
            type=pcl_colors_type
        )

        point_cloud_data = pb2.PointCloudData(
            point_cloud_coords=pcl_coords,
            point_cloud_colors=pcl_colors,
            timestamp=self._getAndFormatTimestamp('camera_time')
        )

        return point_cloud_data
    
    def GetLatestOakdLiteData(self, request, context):
        # Get Camera Data
        camera_data = self.GetLatestImageData(request, context)

        # Get IMU Data
        imu_data = self.GetLatestIMUData(request, context)

        # Get Point Cloud Data
        point_cloud_data = self.GetLatestPointCloudData(request, context)

        oakd_lite_data = pb2.OakdLiteData(
            camera_data=camera_data,
            imu_Data=imu_data,
            point_cloud_data=point_cloud_data,
            timestamp=self._getAndFormatTimestamp('camera_time')
        )

        return oakd_lite_data
    
    def SendMotionCommand(self, command, context):
        motion_type = command.motion_type
        dist = command.distance

        m = self.robot.command_motion_trial([motion_type, dist])
        logger.debug(f"Motion Command: ({motion_type}, {dist}) | M: {m}")
        motion_distance = pb2.MotionDistance(
            left_wheel_dist  =   m[0],
            right_wheel_dist =   m[1]
        )
        return motion_distance

    def GetMotorPositions(self, requests, context):
        motor_id_1_pos, motor_id_2_pos = self.robot.ri.get_motor_positions()
        logger.debug(f"GetMotorPositions: Motor 1: {motor_id_1_pos}, Motor 2: {motor_id_2_pos}")
        motor_positions = pb2.RobotMotorPositions(
            motor_position_left=motor_id_1_pos,
            motor_position_right=motor_id_2_pos,
        )
        return motor_positions
    
    def GetMotorVelocities(self, requests, context):
        motor_id_1_vel, motor_id_2_vel = self.robot.ri.get_motor_velocity()
        logger.debug(f"GetMotorVelocities: Motor 1: {motor_id_1_vel}, Motor 2: {motor_id_2_vel}")
        motor_velocities = pb2.RobotMotorVelocities(
            motor_velocity_left=motor_id_1_vel,
            motor_velocity_right=motor_id_2_vel,
        )
        return motor_velocities


def serve():
    # Create a gRPC server
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    # Add your service to the server
    pb2_grpc.add_RobotServerServicer_to_server(RobotServerServicer(), server)
    # Start the server on a port
    server.add_insecure_port('[::]:50051')
    server.start()
    print("Server running on port 50051. Awaiting requests...")
    try:
        while True:
            time.sleep(86400) # One day
    except KeyboardInterrupt:
        print("Stopping server...")
        server.stop(0)

if __name__ == '__main__':
    serve()
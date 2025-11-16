# For GetLatestLidarData 

# 1. Run Redis Lidar Publisher
# 2. Read Latest Lidar Redis Info and Publish it if GetLatestLidarData is called

# For SendMotionCommand
# 1. Robot Will Use RobotInterface to move the robot according to the command
# 2. Robot will send the distance of its wheel motion back to the laptop

from dxl_controller import DynamixelController
from robot_interface import RobotInterface
from concurrent import futures

import time
import grpc
import generated.robot_data_pb2 as pb2
import generated.robot_data_pb2_grpc as pb2_grpc
import numpy as np
import redis
from robot import Robot

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

        rgb_img = pb2.Image(
            img_bytes=rgb_img_bytes,
            shape=rgb_img_shape,
            type=rgb_img_type
        )

        depth_img = pb2.Image(
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
            accel_x=accel_x,
            accel_y=accel_y,
            accel_z=accel_z,
            gyro_x=gyro_x,
            gyro_y=gyro_y,
            gyro_z=gyro_z,
            timestamp=self._getAndFormatTimestamp(key='imu_time')
        )

        return imu_data
    
    def SendMotionCommand(self, command, context):
        motion_type = command.motion_type
        dist = command.distance

        m = self.robot.command_motion_trial([motion_type, dist])
        
        motion_distance = pb2.MotionDistance(
            left_wheel_dist  =   m[0],
            right_wheel_dist =   m[1]
        )
        return motion_distance


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
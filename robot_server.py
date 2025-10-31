# For GetLatestLidarData 

# 1. Run Redis Lidar Publisher
# 2. Read Latest Lidar Redis Info and Publish it if GetLatestLidarData is called

# For SendMotionCommand
# 1. Robot Will Use RobotInterface to move the robot according to the command
# 2. Robot will send the distance of its wheel motion back to the laptop

from dxl_controller import DynamixelController
from robot_interface import RobotInterface

import grpc
import generated.robot_data_pb2 as pb2
import generated.robot_data_pb2 as pb2_grpc
import numpy as np
import redis

class SendingDataServicer(pb2_grpc.SendingDataServicer):
    """The server implementation of the SendingData service."""
    def __init__(self, device_name='/dev/tty.usbserial-FTAKRMAJ'):
        controller = DynamixelController(device_name=device_name, motor_ids=[1, 2])
        self.ri = RobotInterface(controller=controller)
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0)

    def GetLatestLidarData(self, request, context):
        """Implements the RPC method."""
        print(f"Server received request: success={request.success}, message='{request.message}'")

        # TODO: Not Required, Request is not an input that matters
        if not request.success:
            # You can set an error status if the request indicates failure
            context.set_code(grpc.StatusCode.INVALID_ARGUMENT)
            context.set_details(request.message)
            return pb2.LidarData() # Return an empty response
        
        lidar_data = np.frombuffer(self.redis_client.get("lidar_data")).reshape(-1, 2)

        angles = lidar_data[:, 0]
        dists = lidar_data[:, 1]

        # 1. Create the data to send back
        fake_data = pb2.LidarData(
            angles=angles,
            dists=dists
        )

        # 2. Return the data
        return fake_data
    
    def SendMotionCommand(self, command, context):
        motion_type = command.motion_type
        dist = command.distance

        if motion_type == 'linear':
            m = self.ri.move_dist(dist)
        elif motion_type == 'angular':
            dist = np.deg2rad(dist)
            m = self.ri.rotate_rad(dist)
        
        motion_distance = pb2.MotionDistance(
            left_wheel_dist  =   m[0],
            right_wheel_dist =   m[1]
        )
        return motion_distance



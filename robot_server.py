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

class SendingDataServicer(pb2_grpc.SendingDataServicer):
    """The server implementation of the SendingData service."""
    def __init__(self, device_name='/dev/tty.usbserial-FTAKRMAJ'):
        controller = DynamixelController(device_name=device_name, motor_ids=[1, 2])
        self.ri = RobotInterface(controller=controller)

    def GetLatestLidarData(self, request, context):
        """Implements the RPC method."""
        print(f"Server received request: success={request.success}, message='{request.message}'")

        if not request.success:
            # You can set an error status if the request indicates failure
            context.set_code(grpc.StatusCode.INVALID_ARGUMENT)
            context.set_details(request.message)
            return pb2.FakeData() # Return an empty response

        # 1. Create the data to send back
        fake_data = pb2.FakeData(
            angles=[10.5, 20.1, 30.9, 40.2],
            dists=[5.0, 10.1, 15.3, 20.7]
        )

        # 2. Return the data
        return fake_data
    
    def SendMotionCommand(self, request, context):
        pass



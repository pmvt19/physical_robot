from dxl_controller import DynamixelController
from robot_interface import RobotInterface
import numpy as np
import matplotlib.pyplot as plt
from shapely import Point, Polygon, affinity

import time
import copy
import redis

import rerun as rr

import grpc
import generated.robot_data_pb2 as pb2
import generated.robot_data_pb2_grpc as pb2_grpc

from utils import create_rectangle_geometry

class Robot():
    def __init__(self, device_name='/dev/tty.usbserial-FTAKRMAJ', simulated=False, connection='simulated'):
        self.simulated=simulated
        self.connection = connection
        
        # TODO: Do some design work and see if this is necessary, I'm leaning to not having this, (It's not used internally)
        # TODO: This might be used internally if lidar is simulated as well so might be worth it to keep

        # Initialize Current State (Starts at [x=0.0, y=0.0, theta=0.0])
        self.state = np.array([0.0, 0.0, 0.0])

        # If Robot is not required to be tied to the physical robot, don't initialize the controllers
        # if simulated:
        #     # Initialize RobotInterface with no motor controller
        #     # self.ri = RobotInterface(controller=None)
        #     return
        
        # Initialize Classes For Motor Control
        # self.controller = DynamixelController(device_name=device_name, motor_ids=[1, 2])
        # self.ri = RobotInterface(controller=self.controller)

        
        # Connect to Redis Server for Publishing Lidar Data
        # self.redis_client = redis.Redis(host='localhost', port=6379, db=0)

        if self.connection == 'simulated':
            pass
        elif self.connection == 'client':
            channel = grpc.insecure_channel('192.168.12.155:50051')
            self.stub = pb2_grpc.RobotServerStub(channel)
            print("Motor Logs will appear in the machine where the Robot Server is run")
        elif self.connection == 'physical':
            self.controller = DynamixelController(device_name=device_name, motor_ids=[1, 2])
            self.ri = RobotInterface(controller=self.controller)

            # Connect to Redis Server for Publishing Lidar Data
            self.redis_client = redis.Redis(host='localhost', port=6379, db=0)
        else:
            raise NotImplementedError

    # TODO: DEPRECATE
    def move(self, control, dt):
        # OBSOLETE
        # Ideally, control should be u=[vl,vr]
        pass

    def run_keyboard_control(self):
        """
        See if we can run keyboard control from here if the robot is not simulated
        """
        pass

    # TODO: DEPRECATE
    def read_lidar(self):
        lidar_data = np.frombuffer(self.redis_client.get("lidar_data")).reshape(-1, 2)

        angles = lidar_data[:, 0]
        dist = lidar_data[:, 1]

        rad_angles = (np.pi / 180.0) * angles

        cos = np.cos(rad_angles)
        sin = np.sin(rad_angles)

        x_coords = dist * cos
        y_coords = -dist * sin
        # z_coords = np.zeros_like(x_coords)
        z_coords = np.ones_like(x_coords)
        coords = np.stack((x_coords, y_coords, z_coords), axis=1)
        # coords = np.stack((x_coords, y_coords), axis=1)

        return coords
    
    def _get_single_lidar_reading(self, wait_for_updated_reading):
        if self.connection == 'simulated':
            # DO SOMETHING
            raise NotImplementedError
        else:
            if self.connection == 'client':
                # Make RPC Call here 
                request_ack = pb2.Acknowledge(
                    success=True,
                    message="Client is ready for data!"
                )
                lidar_data = self.stub.GetLatestLidarData(request_ack)

                init_time = lidar_data.timestamp # Move to inside if?
                if wait_for_updated_reading:
                    cur_time = init_time
                    while init_time == cur_time: # TODO: Use is_close?
                        lidar_data = self.stub.GetLatestLidarData(request_ack)
                        cur_time = lidar_data.timestamp
                
                angles = np.array(lidar_data.angles)
                dist = np.array(lidar_data.dists)
                print("Came at time: ", lidar_data.timestamp)
                lidar_data = np.stack((angles, dist), axis=1)

            elif self.connection == 'physical':
                init_time = float(self.redis_client.get('time')) # Move to inside if?
                lidar_data = np.frombuffer(self.redis_client.get("lidar_data")).reshape(-1, 2)
                
                if wait_for_updated_reading:
                    cur_time = init_time
                    while init_time == cur_time: # TODO: Use is_close?
                        cur_time = float(self.redis_client.get('time'))
                        print("Waiting for updated lidar")
                    lidar_data = np.frombuffer(self.redis_client.get("lidar_data")).reshape(-1, 2)

                angles = lidar_data[:, 0]
                dist = lidar_data[:, 1]

            # TODO: Add Fix to Lidar Readings Here
            rad_angles = (np.pi / 180.0) * angles

            cos = np.cos(rad_angles)
            sin = np.sin(rad_angles)

            x_coords = dist * cos
            y_coords = -dist * sin
            z_coords = np.ones_like(x_coords)

            coords = np.stack((x_coords, y_coords, z_coords), axis=1)
            return coords, lidar_data

    def read_lidar_updated(self, manual_verification=False, wait_for_updated_reading=False):
        coords, lidar_data = self._get_single_lidar_reading(wait_for_updated_reading)
        if manual_verification:
            plt.scatter(coords[:, 0], coords[:, 1])
            plt.show()
            user_input = input("Do you want to reread the lidar?")
            while user_input == 'yes':
                coords, lidar_data = self._get_single_lidar_reading(wait_for_updated_reading)
                plt.scatter(coords[:, 0], coords[:, 1])
                plt.show()
                user_input = input("Do you want to reread the lidar?")
        return coords, lidar_data

    # TODO: DEPRECATE
    def read_lidar_trial(self):
        if self.connection == 'simulated':
            raise NotImplementedError
        elif self.connection == 'client':
            # Make RPC Call here 
            # Should I take the raw data? Or make the robot do some conversion for me?
            # raise NotImplementedError
            request_ack = pb2.Acknowledge(
                success=True,
                message="Client is ready for data!"
            )
            lidar_data = self.stub.GetLatestLidarData(request_ack)
            angles = np.array(lidar_data.angles)
            dist = np.array(lidar_data.dists)
            # print(angles)
            # print(dist)
            rad_angles = (np.pi / 180.0) * angles

            cos = np.cos(rad_angles)
            sin = np.sin(rad_angles)

            x_coords = dist * cos
            y_coords = -dist * sin
            # z_coords = np.zeros_like(x_coords)
            z_coords = np.ones_like(x_coords)
            coords = np.stack((x_coords, y_coords, z_coords), axis=1)
            # coords = np.stack((x_coords, y_coords), axis=1)

            return coords
        elif self.connection == 'physical':
            lidar_data = np.frombuffer(self.redis_client.get("lidar_data")).reshape(-1, 2)

            angles = lidar_data[:, 0]
            dist = lidar_data[:, 1]

            rad_angles = (np.pi / 180.0) * angles

            cos = np.cos(rad_angles)
            sin = np.sin(rad_angles)

            x_coords = dist * cos
            y_coords = -dist * sin
            # z_coords = np.zeros_like(x_coords)
            z_coords = np.ones_like(x_coords)
            coords = np.stack((x_coords, y_coords, z_coords), axis=1)
            # coords = np.stack((x_coords, y_coords), axis=1)

            return coords
    
    def motion_command_to_pseudo_motor_diffs(self, motion_command):
        motion_type, pseudo_avg_motion = motion_command

        # Pseudo Motor Diff Examples:
        # [100.0, -100.0], # Forward 100.0 mm relative to heading 
        # [-np.pi/2, -np.pi/2], # Turn in place pi/2 radians CW TODO: CHECK DIRECTION

        if motion_type == 'linear':
            m = np.array([pseudo_avg_motion, -pseudo_avg_motion])
        elif motion_type == 'angular':
            m = np.array([pseudo_avg_motion, pseudo_avg_motion]) # TODO: Check if these need to be negated
        else:
            print("NEED TO RAISE BETTER EXCEPTION")
            raise NotImplementedError
        return m
    
    def predict_state(self, state, motor_position_differential):
        signs = np.sign(motor_position_differential)

        """
        Don't need to break down like this?

        Return: What should we return??
        """

        
        abs_motion = np.abs(motor_position_differential)
        if signs[0] == -1 and signs[1] == -1:
            # Turn In-Place Left
            avg_motion = np.mean(abs_motion)
            motion_type = 'angular'
        elif signs[0] == 1 and signs[1] == 1:
            # Turn In-Place Right
            avg_motion = -1 * np.mean(abs_motion)
            motion_type = 'angular'
        elif signs[0] == 1 and signs[1] == -1:
            # Forward
            avg_motion = np.mean(abs_motion)
            motion_type = 'linear'
        elif signs[0] == -1 and signs[1] == 1:
            # Backward?
            avg_motion = -1 * np.mean(abs_motion)
            motion_type = 'linear'
        else:
            print("Ensure the U2D2 PowerBoard is On")
            raise NotImplementedError

        x, y, theta = state

        if motion_type == 'linear':
            direction_vector = np.array([np.cos(theta), np.sin(theta), 0.0])
            dx_state = direction_vector * avg_motion
            predicted_state = state + dx_state

        elif motion_type == 'angular':
            dx_state = np.array([0.0, 0.0, avg_motion])
            predicted_state = state + dx_state
        else:
            raise NotImplementedError

        return predicted_state

    def command_motion_and_predict_state(self, state, motion_command):
        m = self.command_motion(motion_command)
        predicted_state = self.predict_state(state, m)
        return predicted_state
    
    def request_motion_command_from_user(self):
        """
        Returns: [status, motion_type, angular]
        """
        while True:
            command = input("Enter Robot Motion Command or type \"quit\":\n")

            if command == "quit":
                return ['', 0.0]

            try:
                # TODO: More useful exception messages here
                motion_type, dist = command.split(",")
                dist = float(dist)
                assert(motion_type == 'linear' or motion_type == 'angular')

                return [motion_type, dist]
            except:
                print("That was not a valid command, please try again!")
        
    def get_relative_transformation(self, motor_differentials):
        pass
    
    # TODO: DEPRECATE
    def command_motion(self, motion_command):
        """
        Return motion differential
        """
        motion_type, dist = motion_command
        if self.simulated:
            m = self.motion_command_to_pseudo_motor_diffs(motion_command)
        else:
            # Consider moving this to RobotInterface in a function called "ExecuteMotion or CommandMotion or..."
            if motion_type == 'linear':
                m = self.ri.move_dist(dist)
            elif motion_type == 'angular':
                dist = np.deg2rad(dist)
                m = self.ri.rotate_rad(dist)
            else:
                raise NotImplementedError
        return m

    def command_motion_trial(self, motion_command):
        """
        Return motion differential
        """
        motion_type, dist = motion_command
        if self.connection == 'simulated':
            m = self.motion_command_to_pseudo_motor_diffs(motion_command)
        elif self.connection == 'client':
            # Make RPC Call to robot (For now just return the same m...)
            rpc_motion_command = pb2.MotionCommand(
                motion_type=motion_command[0],
                distance=motion_command[1]
            )
            motion_distance = self.stub.SendMotionCommand(rpc_motion_command)
            m = np.array([motion_distance.left_wheel_dist, motion_distance.right_wheel_dist])

        elif self.connection == 'physical':
            # Consider moving this to RobotInterface in a function called "ExecuteMotion or CommandMotion or..."
            if motion_type == 'linear':
                m = self.ri.move_dist(dist)
            elif motion_type == 'angular':
                m = self.ri.rotate_rad(dist)
            else:
                raise NotImplementedError
        return m
    
    # TODO: Add ability to toggle between grid coords and world coords
    def draw_state(self, ax, state):
        x, y, theta = state

        # Draw Robot Body
        relative_robot_radius = 100 # TODO: Compute this value
        robot_outline = Point([x, y]).buffer(relative_robot_radius)
        ax.fill(*robot_outline.exterior.xy, color='blue')

        # Draw Wheels
        wheel_width = 50
        wheel_height = 20

        wheel_offset = relative_robot_radius + wheel_height/2 # TODO: Compute this value
        left_wheel_y_loc = y + wheel_offset
        right_wheel_y_loc = y - wheel_offset

        

        left_wheel = create_rectangle_geometry(x, left_wheel_y_loc, wheel_width, wheel_height)
        right_wheel = create_rectangle_geometry(x, right_wheel_y_loc, wheel_width, wheel_height)

        left_wheel = affinity.rotate(left_wheel, theta, use_radians=True, origin=[x, y])
        right_wheel = affinity.rotate(right_wheel, theta, use_radians=True, origin=[x, y])
        ax.fill(*left_wheel.exterior.xy, color='black')
        ax.fill(*right_wheel.exterior.xy, color='black')

        # Draw Heading Line
        heading_line_length = 500
        heading_line_ep = np.array([np.cos(theta), np.sin(theta)]) * heading_line_length + np.array([x, y])
        ax.plot([x, heading_line_ep[0]], [y, heading_line_ep[1]], color='red')

    def path_to_motion_commands(self, path):
        raise NotImplementedError
    
    def terminate(self):
        pass
    
if __name__ == "__main__":

    robot = Robot(simulated=False, connection='client')
    # robot.command_motion_trial(['linear', -400])
    # robot.command_motion_trial(['angular', np.pi/2])
    robot.command_motion_trial(['linear', 400])

    # motions = [
    #     ('linear', 400),
    #     ('angular', -np.pi/2),
    #     ('linear', 700),
    # ]
    # for motion_type, dist in motions:
    #     robot.command_motion_trial([motion_type, dist])
    # coords = robot.read_lidar_trial()

    # print(coords)





    

    
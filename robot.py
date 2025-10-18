from dxl_controller import DynamixelController
from robot_interface import RobotInterface
import numpy as np
from shapely import Point

from multiprocessing import Process, Manager
from run_lidar import start_lidar

import time
import redis

import rerun as rr

class Robot():
    def __init__(self, device_name='/dev/tty.usbserial-FTAKRMAJ', simulated=False):
        self.simulated=simulated
        
        # TODO: Do some design work and see if this is necessary, I'm leaning to not having this, (It's not used internally)
        # TODO: This might be used internally if lidar is simulated as well so might be worth it to keep

        # Initialize Current State (Starts at [x=0.0, y=0.0, theta=0.0])
        self.state = np.array([0.0, 0.0, 0.0])

        # If Robot is not required to be tied to the physical robot, don't initialize the controllers
        if simulated:
            # Initialize RobotInterface with no motor controller
            # self.ri = RobotInterface(controller=None)
            return
        
        # Initialize Classes For Motor Control
        self.controller = DynamixelController(device_name=device_name, motor_ids=[1, 2])
        self.ri = RobotInterface(controller=self.controller)

        
        # Connect to Redis Server for Publishing Lidar Data
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0)

    def move(self, control, dt):
        # OBSOLETE
        # Ideally, control should be u=[vl,vr]
        pass

    def keyboard_control(self):
        pass

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
    
    def get_relative_transformation(self, motor_differentials):
        pass
    
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
    
    def draw_state(self, ax, state):
        x, y, theta = state

        robot_outline = Point([x, y]).buffer(100)
        ax.fill(*robot_outline.exterior.xy, color='blue')
        # return robot

    def draw_cosmetic_state(self, ax, state):
        pass
    
    def terminate(self):
        pass
    
if __name__ == "__main__":

    # TODO: Broken DO NOT RUN
    with Manager() as manager:
        shared_dict = manager.dict()
        shared_dict['lidar'] = np.empty((0, 2))
        pub_process = Process(target=start_lidar, args=(shared_dict,))
        pub_process.start()

        time.sleep(10)

        rr.init("3d points", spawn=True)
        start_time = time.time()
        robot = Robot(lidar_data=shared_dict)

        for i in range(10000):
            coords = robot.read_lidar()
            # print(coords)
            rr.set_time("time", duration=time.time()-start_time)
            rr.log("points", rr.Points3D(coords))
            rr.log("points v2", rr.Points3D([[[0.0,0.0,0.0]]], colors=[0, 255, 0], radii=0.1))
            time.sleep(0.1)
        
        pub_process.terminate()
        pub_process.join()

    print("Finished")



    

    
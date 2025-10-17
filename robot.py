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
    def __init__(self, device_name='/dev/tty.usbserial-FTAKRMAJ'):

        # Initialize Classes For Motor Control
        self.controller = DynamixelController(device_name=device_name, motor_ids=[1, 2])
        self.ri = RobotInterface(controller=self.controller)

        # Initialize Current State (Starts at [x=0.0, y=0.0, theta=0.0])
        self.state = np.array([0.0, 0.0, 0.0])

        self.redis_client = redis.Redis(host='localhost', port=6379, db=0)

    def move(self, control, dt):
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



    

    
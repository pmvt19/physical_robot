from dxl_controller import DynamixelController
from robot_interface import RobotInterface
from robot import Robot
from basic_map import BasicMap
import numpy as np
import matplotlib.pyplot as plt

from multiprocessing import Process, Manager
from run_lidar import start_lidar
import time


if __name__ == "__main__":

    with Manager() as manager:
        shared_dict = manager.dict()
        shared_dict['lidar'] = np.empty((0, 2))
        pub_process = Process(target=start_lidar, args=(shared_dict,))
        pub_process.start()

        time.sleep(10)

        start_time = time.time()

        # motions = [
        #     ('linear', 200),
        #     ('angular', -np.pi/2),
        #     ('linear', 200),
        #     ('angular', np.pi/2),
        #     ('linear', 200),
        # ]

        # motions = [
        #     ('linear', 200),
        #     ('angular', -np.pi/4),
        #     ('linear', 500),
        #     ('angular', -np.pi/4),
        #     ('linear', 200),
        # ]

        motions = [
            ('linear', 100),
            ('angular', np.pi/2),
            ('linear', 100),
            ('angular', -np.pi/2),
            ('linear', 100),
        ]

        # Initialization
        robot = Robot(lidar_data=shared_dict)
        robot.ri.set_profile_velocity()
        time.sleep(1.0)
        scan = robot.read_lidar()
        map = BasicMap(initial_scan=scan)

        map.visualize(ax=plt.gca())
        plt.show()

        for motion_type, dist in motions:
            if motion_type == 'linear':
                m = robot.ri.move_dist(dist)
            elif motion_type == 'angular':
                m = robot.ri.rotate_rad(dist)
            else:
                raise NotImplementedError
            
            predicted_state = robot.ri.predict_state(robot.state, m)
            print(f"Predicted State: {predicted_state}")
            robot.state = predicted_state
            scan = robot.read_lidar()
            print(f"Scan Size: {scan.shape}")
            updated_state = map.update(scan, predicted_state)
            print(f"Updated State: {updated_state}")
            robot.state = updated_state

            map.visualize(ax=plt.gca())
            plt.show()

        print(map.points.shape)
        pub_process.terminate()
        pub_process.join()

    print("Finished")
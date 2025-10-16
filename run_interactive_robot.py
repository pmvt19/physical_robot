from dxl_controller import DynamixelController
from robot_interface import RobotInterface
from robot import Robot
from basic_map import BasicMap
from map import Map
import numpy as np
import matplotlib.pyplot as plt
import pickle

from multiprocessing import Process, Manager
from run_lidar import start_lidar
import time


if __name__ == "__main__":


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

    # motions = [
    #         ('linear', 100),
    #         ('angular', -np.pi/2),
    #         ('linear', 100),
    #         # ('angular', -np.pi/2),
    #         # ('linear', 100),
    #     ]

    fl_units = 300
    motions = [
            ('linear', fl_units),
            ('linear', fl_units),
            ('linear', fl_units),
            ('linear', fl_units),
            ('linear', fl_units),
            ('linear', fl_units),
            ('linear', fl_units),
            ('linear', fl_units),
            ('linear', fl_units),
            ('linear', fl_units),
            ('linear', fl_units),
            ('linear', fl_units),
            # ('angular', -np.pi/2),
            # ('linear', 100),
        ]

    # Initialization
    robot = Robot(lidar_data=None)
    robot.ri.set_profile_velocity()
    time.sleep(1.0)
    
    scan = robot.read_lidar()
    # map = BasicMap(initial_scan=scan)
    map = Map(initial_scan=scan)

    map.visualize(ax=plt.gca())
    plt.show()

    # for motion_type, dist in motions:
    i=0
    while True:
        command = input("Enter Robot Motion Command")

        if command == "quit":
            break

        motion_type, dist = command.split(",")
        dist = float(dist)
        print(motion_type, dist)
        # continue
        if motion_type == 'linear':
            m = robot.ri.move_dist(dist)
        elif motion_type == 'angular':
            dist = np.deg2rad(dist)
            m = robot.ri.rotate_rad(dist)
        else:
            raise NotImplementedError
        
        predicted_state = robot.ri.predict_state(robot.state, m)
        print(f"Predicted State: {predicted_state}")
        robot.state = predicted_state
        # scan = robot.read_lidar()
        scan = robot.read_lidar_manual()
        print(f"Scan Size: {scan.shape}")
        updated_state = map.update(scan, predicted_state)
        print(f"Updated State: {updated_state}")
        robot.state = updated_state

        map.visualize(ax=plt.gca())
        plt.show()
        plt.cla()
        map.visualize(ax=plt.gca())
        plt.savefig(f'maps_saved/run6/map_{i}.png')
        i+=1

        pickle.dump(map, open("dump/map_object.pickle", "wb"))
        pickle.dump(map.map, open("dump/map_map.pickle", "wb"))
        pickle.dump(map.get_points(), open("dump/map_points.pickle", "wb"))
    
    print("Finished")
    plt.cla()
    map.visualize(ax=plt.gca())
    plt.savefig("generated_map.png")

    

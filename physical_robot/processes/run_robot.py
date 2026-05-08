# from dxl_controller import DynamixelController
# from robot_interface import RobotInterface
from physical_robot.robot import Robot
# from basic_map import BasicMap
# from map import Map
import numpy as np
import matplotlib.pyplot as plt

from multiprocessing import Process, Manager
# from run_lidar import start_lidar
import time

import cv2


if __name__ == "__main__":

    robot = Robot(connection='client')
    
    # plt.imshow(img[:, :, ::-1])
    # plt.show()

    # while True:
    #     img, depth_img = robot.read_rgb_camera()
    #     cv2.imshow('frame', img)
    #     depth_img = ((depth_img / np.max(depth_img)) * 255).astype(np.uint8)
    #     depth_img = cv2.applyColorMap(depth_img, cv2.COLORMAP_HOT)
    #     # depth_img[depth_img == 0] = 1
    #     cv2.imshow('depth frame', depth_img)
    #     print(np.min(depth_img), np.max(depth_img))
        
    #     if cv2.waitKey(1) == ord('q'):
    #         break
    # exit()

    robot = Robot(connection='client')

    while True:
        motion_command = robot.request_motion_command_from_user()
        if motion_command[0] == '': # No Motion Command
            break
        m = robot.command_motion_trial(motion_command)
    exit()

    # while True:
    #     _, _ = robot.read_lidar_updated(wait_for_updated_reading=True, manual_verification=True)
        
    # exit()


    start_time = time.time()

    # motions = [
    #     ('linear', 400),
    #     ('angular', -np.pi/2),
    #     ('linear', 700),
    # ]

    # Initialization
    robot = Robot(connection='client')
    # robot.ri.set_profile_velocity()
    time.sleep(1.0)

    # path = np.array([[-352, -456, 6.21],
    #                  [0, 0, 0],
    #                  [500, 0, 0],
    #                  [500, 500, 0],
    #                  [1000, 1000, 0],
    #                  [2000, 1000, 0],
    #                  [2000, 2000, 0],
    #                  [2600, 2500, 0],
    #                  [2600, 3000, 0],])

    path = np.array([[520.75542235, -551.41878351, 4.83186841],
                     [0, 0, 0],
                     [-1495, -445, 0],
                     [-2433, -700, 0],
                     [-2335, -2493, 0],])

    motions = robot.path_to_motion_commands(path)

    scan, raw_data = robot.read_lidar_updated(wait_for_updated_reading=True)
    # map = BasicMap(initial_scan=scan)
    # map = Map(initial_scan=scan)

    # map.visualize(ax=plt.gca())
    # plt.show()
    np.set_printoptions(suppress=True)

    for motion_type, dist in motions:
        m = robot.command_motion_trial([motion_type, dist])
        
        # predicted_state = robot.predict_state(robot.state, m)
        # print(f"Predicted State: {np.round(predicted_state, 2)}")
        # robot.state = predicted_state
        # scan, raw_data = robot.read_lidar_updated(wait_for_updated_reading=True)
        # print(f"Scan Size: {scan.shape}")
        # updated_state = map.update(scan, predicted_state)
        # print(f"Updated State: {updated_state}")
        # robot.state = updated_state

        # map.visualize(ax=plt.gca())
        # plt.show()
    # map.visualize(ax=plt.gca())
    # plt.show()


    print("Finished")
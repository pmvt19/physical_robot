from robot import Robot
from basic_map import BasicMap
from map import Map
import numpy as np
import matplotlib.pyplot as plt
import pickle
import time


# TODO: Rename this file and function?

def run_interactive_robot():
    pass

# TODO BUG HACK FIXME: RERUN THIS SCRIPT!!!

if __name__ == "__main__":

    run_num = 1
    dump_dir = f'dumps/run{run_num}'
    imgs_dir = f'maps_saved/run{run_num}'


    start_time = time.time()

    # Initialization
    robot = Robot(connection='client')
    
    scan, _ = robot.read_lidar_updated(manual_verification=True, wait_for_updated_reading=True)

    map = Map(initial_scan=scan)

    map.visualize(ax=plt.gca())
    plt.show()

    # for motion_type, dist in motions:
    i=0
    while True:
        # command = input("Enter Robot Motion Command")

        # if command == "quit":
        #     break

        # motion_type, dist = command.split(",")
        # dist = float(dist)
        # print(motion_type, dist)
        motion_command = robot.request_motion_command_from_user()
        if motion_command[0] == '': # No Motion Command:
            break
        # continue
        m = robot.command_motion_trial(motion_command)
        print(robot.state)
        
        predicted_state = robot.predict_state(robot.state, m)
        print(robot.state)
        print(f"Predicted State: {predicted_state}")
        robot.state = predicted_state
        # scan = robot.read_lidar()
        print(robot.state)
        scan, _ = robot.read_lidar_updated(manual_verification=False, wait_for_updated_reading=True)
        print(f"Scan Size: {scan.shape}")
        updated_state = map.update(scan, predicted_state)
        print(robot.state)
        print(f"Updated State: {updated_state}")
        robot.state = updated_state

        map.visualize(ax=plt.gca())
        plt.show()
        plt.cla()
        map.visualize(ax=plt.gca())
        plt.savefig(f'{imgs_dir}/map_{i}.png')
        i+=1

        pickle.dump(map, open(f"{dump_dir}/map_object.pickle", "wb"))
        pickle.dump(map.map, open(f"{dump_dir}/map_map.pickle", "wb"))
        pickle.dump(map.get_points(), open(f"{dump_dir}/map_points.pickle", "wb"))
    
    print("Finished")
    plt.cla()
    map.visualize(ax=plt.gca())
    plt.savefig("generated_map.png")

    

from robot import Robot
from basic_map import BasicMap
from map import Map
import numpy as np
import matplotlib.pyplot as plt
import pickle
import time

# from config import scene_name

"""
saves/
    -scenes/
        -scene_name/
            -map.pickle
            -prm.pickle
            -incremental_imgs/


"""


# TODO: Rename this file and function?

def run_interactive_robot():
    pass

def init_directories():
    pass

if __name__ == "__main__":

    ## TODO: Will update directory structure soon
    run_num = 6
    dump_dir = f'dumps/run{run_num}'
    imgs_dir = f'maps_saved/run{run_num}'

    scene_name = 'tmp'
    map_save_dir = f'saves/scenes/{scene_name}'
    ## TODO: Will update directory structure soon


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
        motion_command = robot.request_motion_command_from_user()
        if motion_command[0] == '': # No Motion Command
            break
        # continue
        m = robot.command_motion_trial(motion_command)
        print(robot.state)
        
        predicted_state = robot.predict_state(robot.state, m)
        print(robot.state)
        print(f"Predicted State: {predicted_state}")
        robot.state = predicted_state

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
        # plt.savefig(f'{imgs_dir}/map_{i}.png')
        plt.savefig(f'{map_save_dir}/map_imgs/map_{i}.png')
        i+=1

        pickle.dump(map, open(f"{dump_dir}/map_object.pickle", "wb"))
        pickle.dump(map.map, open(f"{dump_dir}/map_map.pickle", "wb"))
        pickle.dump(map.get_points(), open(f"{dump_dir}/map_points.pickle", "wb"))
    
    print("Finished")
    plt.cla()
    map.visualize(ax=plt.gca())
    plt.savefig("generated_map.png")

    

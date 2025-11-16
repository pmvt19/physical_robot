from robot import Robot
from map import Map
import numpy as np
import matplotlib.pyplot as plt
import pickle
from particle_filter import ParticleFilter
import time


if __name__ == "__main__":
    start_time = time.time()

    # Initialization
    robot = Robot(connection='client')
    time.sleep(1.0)
    
    # Load Premade Map
    # map : Map = pickle.load(open("dumps/run0/map_object.pickle", "rb"))
    # map : Map = pickle.load(open("dumps/run1/map_object.pickle", "rb")) # Map of full apartment
    # map : Map = pickle.load(open("dumps/run3/map_object.pickle", "rb")) # Map of just bedroom (made with updated robot_interface (inverse angular directions))
    # map : Map = pickle.load(open("dumps/run4/map_object.pickle", "rb")) # Map of just kitchen
    # map : Map = pickle.load(open("dumps/run5/map_object.pickle", "rb")) # Map of just kitchen (With Filtered Lidar)
    map : Map = pickle.load(open("saves/scenes/tmp/map/map_object.pickle", "rb")) # Map of Apartment

    # Create Particle Filter Object
    pf = ParticleFilter(map_obj=map)
    pf.initialize(num_particles=10000)

    # Visualize Current Map
    pf.visualize_particles(plt.gca())
    pf.map.visualize_points(plt.gca())
    plt.scatter(robot.state[0], robot.state[1], color='orange', zorder=2)
    plt.show()

    map.visualize(ax=plt.gca())
    plt.show()

    # for motion_type, dist in motions:
    while True:
        motion_command = robot.request_motion_command_from_user()
        if motion_command[0] == '': # No Motion Command:
            break
        m = robot.command_motion_trial(motion_command)
        
        predicted_state = robot.predict_state(robot.state, m)
        print(f"Predicted State: {predicted_state}")
        robot.state = predicted_state

        scan, lidar_data = robot.read_lidar_updated(wait_for_updated_reading=True)

        print(f"Scan Size: {scan.shape} | Lidar Data Size: {lidar_data.shape}")

        updated_state = pf.step(motion_delta=motion_command, scan=lidar_data) # SCAN IS CURRENTLY WRONG!!!!
        print(f"Updated State: {updated_state}")
        robot.state = updated_state

        pf.visualize_particles(plt.gca())
        pf.map.visualize_points(plt.gca())
        # plt.scatter(scan[:, 0], scan[:, 1], color='purple')
        plt.scatter(robot.state[0], robot.state[1], color='orange', zorder=2)
        plt.show()
    
    
    print("Finished")



    

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from scipy import ndimage, stats
from map import Map
from particle_filter import ParticleFilter
from simulate_lidar import SimulatedLidar
from robot import Robot

from utils import pairwise_dists
from test_utils import generate_fake_map, load_saved_map

np.set_printoptions(suppress=True)

if __name__ == '__main__':
    seed = np.random.randint(10000)
    print(f"Using Seed: {seed}")
    seed = 5770 # Using this seed to figure out why expanding map doesn't succeed on the first try
    np.random.seed(seed)
    mymap = load_saved_map()
    # mymap = load_saved_map(f"saves/scenes/tmp/map/")
    # mymap = generate_fake_map()
    mymap.visualize(plt.gca())
    plt.show()

    sl = SimulatedLidar(mymap, 100, 10000)

    pf = ParticleFilter(mymap)
    # pf._compute_dist_map()
    # pf.generate_initial_particles(num_particles=100000)
    pf.initialize(num_particles=10000)
    pf.visualize_particles(plt.gca())
    mymap.visualize_points(plt.gca())
    plt.show()
    
    # GOOD TESTER MOTIONS
    mds = [
        ['linear', 100.0], # Forward relative to heading 
    ] * 40 + \
    [
        ['angular', -np.pi/2], # Turn in place CCW
    ] + \
    [
        ['linear', 100.0], # Forward relative to heading 
    ] * 40

    # mds = [
    #     ['linear', 500.0], # Forward relative to heading 
    # ] * 40 

    # [np.pi/2, np.pi/2], # Turn in place CW
    robot = Robot(connection='simulated')
    robot.state = np.array([-1000.0, 2500.0, 3*np.pi/2])
    for i in range(len(mds)):
        motion_delta = mds[i]
        print("singular motion_delta", motion_delta)

        # new_state = new_state + motion_delta
        # translated_rotated_points, line_segment_eps, map_points, angles, r_dists, unrotated_points, r_angles_local = sl.simulate_lidar(loc=new_state)
        translated_rotated_points, line_segment_eps, map_points, angles, r_dists, unrotated_points, r_angles_local = sl.simulate_lidar(loc=robot.state)
        scan_v2 = np.stack((r_angles_local, r_dists), axis=1)
        state_estimate = pf.step(motion_delta=motion_delta, scan=scan_v2)
        
        print(f"Actual State: {np.round(robot.state, 2)}")
        print(f"State Estimate: {np.round(state_estimate, 2)}")
        print(f"State Difference: {np.round(robot.state - state_estimate, 2)}")
        # motor_diffs = robot.command_motion(motion_delta)
        # robot.state = robot.predict_state(robot.state, motor_diffs)
        robot.state = robot.command_motion_and_predict_state(robot.state, motion_delta)
        

        pf.visualize_particles(plt.gca())
        pf.map.visualize_points(plt.gca())
        # plt.scatter(translated_rotated_points[:, 0], translated_rotated_points[:, 1], color='purple')
        plt.scatter(unrotated_points[:, 0], unrotated_points[:, 1])
        plt.scatter(robot.state[0], robot.state[1], color='orange', zorder=2)
        robot.draw_state(plt.gca(), robot.state)
        # plt.xlim(-2000, 5000)
        # plt.ylim(-2000, 5000)
        plt.show()
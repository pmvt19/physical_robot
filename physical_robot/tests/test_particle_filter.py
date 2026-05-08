import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from scipy import ndimage, stats
from map import Map
from particle_filter import ParticleFilter
from simulate_lidar import SimulatedLidar

from utils import pairwise_dists
from test_utils import generate_fake_map, load_saved_map

if __name__ == '__main__':
    mymap = load_saved_map()
    sl = SimulatedLidar(mymap, 100, 10000)

    pf = ParticleFilter(mymap)
    pf._compute_dist_map()
    pf.generate_initial_particles(num_particles=10000)
    
    new_state = np.array([0.0, 0.0, 0.0])
    pf.visualize_particles(plt.gca())
    pf.map.visualize_points(plt.gca())
    plt.scatter(new_state[0], new_state[1], color='orange')
    plt.show()

    mds = [
        [100.0, 0.0, 0.0] 
    ] * 8 + [
        [0.0, 100.0, 0.0]
    ] * 15 + [
        [100.0, 0.0, 0.0]
    ] * 10
    np.set_printoptions(suppress=True)
    for i in range(len(mds)):
        motion_delta = np.array(mds[i])

        new_state = new_state + motion_delta
        translated_rotated_points, line_segment_eps, map_points, angles, r_dists, unrotated_points, r_angles_local = sl.simulate_lidar(loc=new_state)
        scan_v2 = np.stack((r_angles_local, r_dists), axis=1)
        state_estimate = pf.step(motion_delta=motion_delta, scan=scan_v2)
        print(f"Actual State: {np.round(new_state, 2)}")
        print(f"State Estimate: {np.round(state_estimate, 2)}")

        pf.visualize_particles(plt.gca())
        pf.map.visualize_points(plt.gca())
        plt.scatter(new_state[0], new_state[1], color='orange', zorder=2)
        plt.show()
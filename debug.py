import numpy as np
import matplotlib.pyplot as plt

from simulate_lidar import SimulatedLidar
from test_utils import generate_fake_map

if __name__ == '__main__':
    mymap = generate_fake_map()
    # state = np.array([-1000.0, 2500.0, np.pi/2])
    state = np.array([-1000.0, 2500.0, 0])

    sl = SimulatedLidar(mymap, 100, 10000)

    translated_rotated_points, line_segment_eps, map_points, r_angles, r_dists, unrotated_points, r_angles = sl.simulate_lidar(loc=state)

    sl.animate_lidar(state)

    plt.clf()
    mymap.visualize_points(plt.gca())
    plt.scatter(translated_rotated_points[:, 0], translated_rotated_points[:, 1])
    plt.axis('equal')
    plt.show()
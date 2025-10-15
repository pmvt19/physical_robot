import numpy as np
import matplotlib.pyplot as plt
from scipy import ndimage

from simulate_lidar import SimulatedLidar

from utils import pairwise_dists
from test_utils import generate_fake_map

class ParticleFilter():
    def __init__(self, map_obj):
        self.map = map_obj

    def _compute_dist_map(self):
        inverse_map = 1 - self.map.map
        self.dist_map = ndimage.distance_transform_edt(inverse_map)

    # TODO: Change function name - Done?
    def batch_get_measurement_update(self, states, scan_actual):
        # States: (N, 3)
        # Scan_Actual: (M, 2)
        # Ideally, scan_actual[i] is just (angle, dist)
        N, d = states.shape

        angles = scan_actual[:, 0] # (M,)
        point_dists = scan_actual[:, 1] # (M,)

        state_headings = states[:, 2] # (N,)
        offset_angles = angles.reshape(-1, 1) + state_headings.reshape(1, -1) # (M, 1) + (1, N) = (M, N)

        coses = np.cos(offset_angles) # (M, N)
        sines = np.sin(offset_angles) # (M, N)
        vecs = np.stack((coses, sines), axis=2) # (M, N, 2)

        origin_centered_points = vecs * point_dists.reshape(-1, 1, 1) # (M, N, 2) * (M,) = (M, N, 2) (MIGHT NEED TO RESHAPE point_dists)
        batch_simulated_lidar_readings = origin_centered_points + states[:, :2].reshape(1, N, -1) # (M, N, 2) + (1, N, 2) = (M, N, 2) (Probably need to transform states a bit)
        batch_simulated_lidar_readings = batch_simulated_lidar_readings.transpose(1, 0, 2) # Transpose the Matrix to be (N, M, 2) {I think this makes more sense, but I already implemented this function...}
        return batch_simulated_lidar_readings


    def visualize_dist_map(self, ax):
        ax.imshow(np.rot90(self.dist_map))
    
    def visualize_map(self, ax):
        self.map.visualize(ax)


if __name__ == '__main__':
    mymap = generate_fake_map()

    state = np.array([-1000.0, 2500.0, np.pi/2])

    sl = SimulatedLidar(mymap, 100, 10000)
    _, _, _, angles, r_dists = sl.simulate_lidar(loc=state)
    # Format Lidar Readings:
    print(angles.shape, r_dists.shape)
    scan = np.stack((angles, r_dists), axis=1)
    print(scan.shape)


    pf = ParticleFilter(mymap)
    pf._compute_dist_map()

    fig, (ax1, ax2) = plt.subplots(1, 2)
    pf.visualize_map(ax1)
    pf.visualize_dist_map(ax2)
    plt.show()

    pf.batch_get_measurement_update(state.reshape(-1, 3), scan)


    
        
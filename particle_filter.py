import numpy as np
import matplotlib.pyplot as plt
from scipy import ndimage, stats
from map import Map

from simulate_lidar import SimulatedLidar

from utils import pairwise_dists
from test_utils import generate_fake_map, load_saved_map

class ParticleFilter():
    def __init__(self, map_obj, scale_factor=10):
        self.map : Map = map_obj
        self.scale_factor = scale_factor
        print("WARNING: GUESS ON THE SCALE FACTOR!!")

        self.normal_distribution = stats.norm(loc=0, scale=self.scale_factor)

    def _compute_dist_map(self):
        inverse_map = 1 - self.map.map
        self.dist_map = ndimage.distance_transform_edt(inverse_map)

    def _batch_get_simulated_lidar(self):
        raise NotImplementedError

    def _batch_get_probabilities(self):
        raise NotImplementedError

    # TODO: Change function name - Done?
    def batch_get_measurement_update(self, states, scan_actual, tmp_points=None):
        # States: (N, 3)
        # Scan_Actual: (M, 2)
        # Ideally, scan_actual[i] is just (angle, dist)

        ### ---- Get Batch Simulated Lidar ---- ###
        N, d = states.shape

        angles = scan_actual[:, 0] # (M,)
        point_dists = scan_actual[:, 1] # (M,)

        state_headings = states[:, 2] # (N,)

        ### TODO: CHECK THIS: TODO ###
        offset_angles = angles.reshape(-1, 1) - (np.pi/2 - state_headings.reshape(1, -1)) # (M, 1) + (1, N) = (M, N)
        ### TODO: CHECK THIS: TODO ###
        
        # print(np.rad2deg(angles % (2*np.pi)), state_headings)
        # exit()

        coses = np.cos(offset_angles) # (M, N)
        sines = np.sin(offset_angles) # (M, N)
        vecs = np.stack((coses, sines), axis=2) # (M, N, 2)

        origin_centered_points = vecs * point_dists.reshape(-1, 1, 1) # (M, N, 2) * (M,) = (M, N, 2) (MIGHT NEED TO RESHAPE point_dists)
        batch_simulated_lidar_readings = origin_centered_points + states[:, :2].reshape(1, N, -1) # (M, N, 2) + (1, N, 2) = (M, N, 2) (Probably need to transform states a bit)
        batch_simulated_lidar_readings = batch_simulated_lidar_readings.transpose(1, 0, 2) # Transpose the Matrix to be (N, M, 2) {I think this makes more sense, but I already implemented this function...}

        plt.scatter(batch_simulated_lidar_readings[0, :, 0], batch_simulated_lidar_readings[0, :, 1], label='sim lidar')
        plt.scatter(tmp_points[:, 0], tmp_points[:, 1], label='scanned lidar')
        self.map.visualize_points(plt.gca())
        plt.legend()
        plt.show()
        ### ---- Get Batch Simulated Lidar ---- ###

        ### ---- Get Probabilities ---- ###
        flattened_batch_simulated_lidar_readings = batch_simulated_lidar_readings.reshape(-1, 2) # (N, M, 2) -> (N*M, 2)
        flattened_batch_grid_coords = self.map.batch_world_to_grid_coords(flattened_batch_simulated_lidar_readings) # (N*M, 2)
        flattened_batch_dists = self.dist_map[flattened_batch_grid_coords[:, 0], flattened_batch_grid_coords[:, 1]] # (N*M,)
        print(flattened_batch_dists)
        flattened_batch_probs = self.normal_distribution.pdf(flattened_batch_dists) # (N*M,)
        batch_probs = flattened_batch_probs.reshape(N, -1) # (N, M)
        ### ---- Get Probabilities ---- ###

        return batch_probs, batch_simulated_lidar_readings


    def visualize_dist_map(self, ax):
        ax.imshow(np.rot90(self.dist_map))
    
    def visualize_map(self, ax):
        self.map.visualize(ax)


if __name__ == '__main__':
    mymap = generate_fake_map()
    # mymap = load_saved_map()

    

    state = np.array([-1000.0, 2500.0, np.pi/2])

    ### ---- Used for Testing ---- ###
    # state = np.array([-1000.0, 2500.0, np.pi/4])
    # state = np.array([-1000.0, 2500.0, 3*np.pi/4])
    # state = np.array([-1000.0, 2500.0, 3*np.pi/2])
    # state = np.array([-1000.0, 2500.0, 5*np.pi/2])
    # state = np.array([-1000.0, 2500.0, 0])

    sl = SimulatedLidar(mymap, 100, 10000)
    translated_rotated_points, line_segment_eps, map_points, angles, r_dists, unrotated_points, r_angles_local = sl.simulate_lidar(loc=state)
    # Format Lidar Readings:
    print(translated_rotated_points.shape, line_segment_eps.shape, map_points.shape)
    print(angles.shape, r_dists.shape)
    scan = np.stack((angles, r_dists), axis=1)
    scan_v2 = np.stack((r_angles_local, r_dists), axis=1)
    print(np.rad2deg(angles))
    print(np.rad2deg(r_angles_local) % 360)
    # exit()
    print("Scan Shapes")
    print(scan.shape, scan_v2.shape)


    pf = ParticleFilter(mymap)
    pf._compute_dist_map()

    fig, (ax1, ax2) = plt.subplots(1, 2)
    pf.visualize_map(ax1)
    pf.visualize_dist_map(ax2)
    plt.show()
    
    
    batch_probs, batch_simulated_lidar_readings = pf.batch_get_measurement_update(state.reshape(-1, 3), scan_v2, translated_rotated_points)
    print(batch_probs)


    
        
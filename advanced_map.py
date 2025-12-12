import numpy as np
from map import Map
from utils import point_segment_distance, timer, line_seg_to_points_dist
from utils_parallel import parallel_point_segment_distance
from test_utils import generate_fake_scan
import math
import matplotlib.pyplot as plt
from skimage.draw import line
from icp import run_icp

FREE = 0
OCCUPIED = 1

class AdvancedMap(Map):
    def __init__(self, resolution=10.0, origin=None):
        super().__init__(resolution=resolution, origin=origin)

        N, M, *_ = self.map.shape
        self.map = np.zeros((N, M, 2))

    def init_map(self, initial_scan):
        initial_state = np.array([0.0, 0.0, 0.0])
        self.update_map(initial_scan, initial_state)

    def update(self, scan, predicted_state):
        T = run_icp(scan, self.get_points(), predicted_state, visualize=False)
        updated_theta = np.arctan2(T[1,0],T[0,0]) % (2*np.pi)

        updated_x = T[0, 2]
        updated_y = T[1, 2]

        aligned_scan = (T@scan.T).T

        updated_state = np.array([updated_x, updated_y, updated_theta])

        self.update_map(aligned_scan, updated_state)
        return updated_state

    @timer
    def update_map(self, aligned_scan, updated_state):
        # Compute Aligned Lidar Beams
        # Update all Cells which the lidar beam passes through to be empty

        line_segments_grid = self.batch_world_to_grid_coords(aligned_scan)
        state_grid_coords = self.world_to_grid_coords(updated_state[:2])

        for line_segment in line_segments_grid:
            rr, cc = line(state_grid_coords[0], state_grid_coords[1], line_segment[0], line_segment[1])
            self.map[rr[:-1], cc[:-1], FREE] += 1

        # TODO: Integrate Properly
        idxes = self.batch_world_to_grid_coords(aligned_scan)
        is_valid = self.validate_map_boundaries(idxes)
        # TODO: Clean this up if correct
        if is_valid:
            self.map[idxes[:, 0], idxes[:, 1], OCCUPIED] += 1
        else:
            self.expand_map(idxes)
            idxes = self.batch_world_to_grid_coords(aligned_scan)
            self.map[idxes[:, 0], idxes[:, 1], OCCUPIED] += 1

    def expand_map(self, req_grid_coords):
        free_layer_coords_and_values, occupied_layer_coords_and_values = self.get_points_and_values_by_layer()

        ### --- Expand Map Here --- ###
        map_size_discretized = self._compute_new_map_size(grid_coords=req_grid_coords)
        map_size_discretized = map_size_discretized.astype(np.int32)
        N, M = map_size_discretized
        print(f"Expanded Map Discritized Size: {map_size_discretized}")
        self.map = np.zeros(N, M, 2)
        self.mx = self.map.shape[0] // 2
        self.my = self.map.shape[1] // 2
        ### --- Expand Map Here --- ###

        new_grid_coords_free_layer = self.batch_world_to_grid_coords(free_layer_coords_and_values[:, :2])
        new_grid_coords_occupied_layer = self.batch_world_to_grid_coords(occupied_layer_coords_and_values[:, :2])

        self.map[new_grid_coords_free_layer[:, 0], new_grid_coords_free_layer[:, 1], FREE] = free_layer_coords_and_values[:, 2]
        self.map[new_grid_coords_occupied_layer[:, 0], new_grid_coords_occupied_layer[:, 1], OCCUPIED] = new_grid_coords_occupied_layer[:, 2]

    def get_points(self, threshold=0.5):
        probability_map = self.map_to_probability_map()
        idxes = np.where(probability_map > threshold)
        xs, ys = idxes
        pc_idxes = np.stack((xs, ys), axis=1)
        pc_coords = self.batch_grid_to_approx_world_coords(pc_idxes)
        return pc_coords
    
    def map_layer_to_coords_and_values(self, map_layer : np.ndarray):
        idxes = np.where(map_layer> 0)
        xs, ys = idxes
        pc_idxes = np.stack((xs, ys), axis=1)
        pc_coords = self.batch_grid_to_approx_world_coords(pc_idxes)
        pc_values = map_layer.reshape(-1, 1)
        pc_coords_and_values = np.hstack((pc_coords, pc_values))
        return pc_coords_and_values

    def get_points_and_values_by_layer(self):
        free_layer_coords_and_values = self.map_layer_to_coords_and_values(self.map[:, :, FREE])
        occupied_layer_coords_and_values = self.map_layer_to_coords_and_values(self.map[:, :, OCCUPIED])
        return free_layer_coords_and_values, occupied_layer_coords_and_values
    
    def map_to_probability_map(self):
        return (self.map[:, :, OCCUPIED] + 1) / (np.sum(self.map, axis=2) + 2)
    
    def visualize(self, ax):
        probability_map = self.map_to_probability_map()
        ax.imshow((np.rot90(probability_map)))
        
if __name__ == '__main__':
    advanced_map = AdvancedMap()
    advanced_map.init_map(initial_scan=generate_fake_scan())
    advanced_map.visualize(plt.gca())
    plt.show()


    
import numpy as np
from physical_robot.maps import Map
from physical_robot.utils import point_segment_distance, timer, line_seg_to_points_dist
# from test_utils import generate_fake_scan
import math
import matplotlib.pyplot as plt
from skimage.draw import line
from physical_robot.algorithms.icp import run_icp
from scipy.ndimage import binary_dilation, gaussian_filter, median_filter
from sklearn.neighbors import KDTree
from physical_robot.utils import create_circular_kernel
from scipy.ndimage import grey_dilation
FREE = 0
OCCUPIED = 1

class AdvancedMap(Map):
    def __init__(self, resolution=10.0, origin=None):
        super().__init__(resolution=resolution, origin=origin)

        N, M, *_ = self.map.shape
        self.map = np.zeros((N, M, 2))
        self.map_type_name = 'advanced_map'

        self.needs_inflation_update = True # TODO: Deprecate this field
        self.current_inflation_radius = 0

        self.cached_inflated_map = None

    def init_map(self, initial_scan):
        initial_state = np.array([0.0, 0.0, 0.0])
        self.update_map(initial_scan, initial_state)
    
    def get_shape_2d(self):
        N, M, *_ = self.map.shape
        return (N, M)

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
        
        self.needs_inflation_update = True
        self.current_inflation_radius = 0

    def expand_map(self, req_grid_coords):
        free_layer_coords_and_values, occupied_layer_coords_and_values = self.get_points_and_values_by_layer()

        ### --- Expand Map Here --- ###
        map_size_discretized = self._compute_new_map_size(grid_coords=req_grid_coords)
        map_size_discretized = map_size_discretized.astype(np.int32)
        N, M = map_size_discretized
        print(f"Expanded Map Discritized Size: {map_size_discretized}")
        self.map = np.zeros((N, M, 2))
        self.mx = self.map.shape[0] // 2
        self.my = self.map.shape[1] // 2
        ### --- Expand Map Here --- ###

        new_grid_coords_free_layer = self.batch_world_to_grid_coords(free_layer_coords_and_values[:, :2])
        new_grid_coords_occupied_layer = self.batch_world_to_grid_coords(occupied_layer_coords_and_values[:, :2])

        self.map[new_grid_coords_free_layer[:, 0], new_grid_coords_free_layer[:, 1], FREE] = free_layer_coords_and_values[:, 2]
        self.map[new_grid_coords_occupied_layer[:, 0], new_grid_coords_occupied_layer[:, 1], OCCUPIED] = occupied_layer_coords_and_values[:, 2]
        self.needs_inflation_update = True
        self.current_inflation_radius = 0

    def get_points(self, threshold=0.5):
        probability_map = self.map_to_probability_map()
        idxes = np.where(probability_map > threshold)
        xs, ys = idxes
        pc_idxes = np.stack((xs, ys), axis=1)
        pc_coords = self.batch_grid_to_approx_world_coords(pc_idxes)
        return pc_coords

    def get_map_2d(self):
        return self.map_to_probability_map()
    
    def map_layer_to_coords_and_values(self, map_layer : np.ndarray):
        idxes = np.where(map_layer > 0)
        xs, ys = idxes
        pc_idxes = np.stack((xs, ys), axis=1)
        pc_coords = self.batch_grid_to_approx_world_coords(pc_idxes)
        pc_values = map_layer[xs, ys].reshape(-1, 1)
        pc_coords_and_values = np.hstack((pc_coords, pc_values))
        return pc_coords_and_values

    def get_points_and_values_by_layer(self):
        free_layer_coords_and_values = self.map_layer_to_coords_and_values(self.map[:, :, FREE])
        occupied_layer_coords_and_values = self.map_layer_to_coords_and_values(self.map[:, :, OCCUPIED])
        return free_layer_coords_and_values, occupied_layer_coords_and_values
    
    def map_to_probability_map(self):
        return (self.map[:, :, OCCUPIED] + 1) / (np.sum(self.map, axis=2) + 2)

    def inflate_obstacles(self, inflation_radius=210):

        self.inflated_map = self.map.copy()
        kernel_size = int(inflation_radius // self.resolution) # Hard Coded to the radius of the robot

        kernel = create_circular_kernel(kernel_size, kernel_size/2)

        self.inflated_map[:, :, OCCUPIED] = grey_dilation(self.inflated_map[:, :, OCCUPIED], footprint=kernel)
        self.inflated_map[self.inflated_map[:, :, OCCUPIED] > 0, FREE] = 0

        self.needs_inflation_update = False
        self.current_inflation_radius = inflation_radius

    def get_inflated_map_2d(self, inflation_radius=210):
        assert(inflation_radius > 0)
        if self.current_inflation_radius != inflation_radius:
            self.inflate_obstacles(inflation_radius=inflation_radius)
            self.cached_inflated_map = (self.inflated_map[:, :, OCCUPIED] + 1) / (np.sum(self.inflated_map, axis=2) + 2)
        return self.cached_inflated_map
        # return (self.inflated_map[:, :, OCCUPIED] + 1) / (np.sum(self.inflated_map, axis=2) + 2)

    def get_frontier_candidates(self, do_smoothing=False):
        probablity_map = self.get_map_2d()

        raw_map = self.map.copy()

        if do_smoothing:
            raw_map[:, :, FREE] = gaussian_filter(raw_map[:, :, FREE], sigma=1.0)
            raw_map[:, :, OCCUPIED] = gaussian_filter(raw_map[:, :, OCCUPIED], sigma=1.0)

        probablity_map = (raw_map[:, :, OCCUPIED] + 1) / (np.sum(raw_map, axis=2) + 2)

        unknown_cells = probablity_map == 0.5
        free_cells = probablity_map < 0.5

        dilated_unknown_cells = binary_dilation(unknown_cells)

        frontier_cells = np.logical_and(dilated_unknown_cells, free_cells)

        return frontier_cells

    def is_valid_frontier_cluster(self, cluster_id, cluster_sizes):
        # TODO: Make this a function that defines valid frontiers
        # For example, 5 is hard coded to cell sizes right now
        # What if we want it to find the area of extent of the frontier?
        # We can get the axis aligned rectangle by taking the boundary points
        # and computing the area and this area should be above a certain threshold


        # If you want to enfoce a fixed area size
        metric_size_threshold_mm_sq = 100
        #...


        # TODO: Also, if you want to enforce a fixed length, it should be in metric
        # units, not in cell units as the resolution can change

        # If you want to enforce a fixed cumulative size
        metric_size_threshold_mm = 100
        cell_size_threshold = metric_size_threshold_mm / self.resolution
        return cluster_sizes[cluster_id] > cell_size_threshold
    
        

    def get_frontiers(self):
        frontier_cells = self.get_frontier_candidates()
        # frontier_cells = median_filter(frontier_cells, size=3)

        # Get Frontier Cells Coords
        xs, ys = np.where(frontier_cells > 0)
        
        cluster_labels = np.zeros(self.get_shape_2d())

        N, M = cluster_labels.shape
        neighbors = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]
        label_id = 1
        # for i in range(N):
        #     for j in range(M):
        cluster_sizes = {}
        for i, j in zip(xs, ys):
            if frontier_cells[i, j] == 1:
                q = [(i, j)]
                cluster_sizes[label_id] = 1
                while q:
                    x, y = q.pop(0)

                    if cluster_labels[x, y] > 0:
                        continue
                    cluster_labels[x, y] = label_id
                    cluster_sizes[label_id] += 1

                    for ox, oy in neighbors:
                        nx = x + ox
                        ny = y + oy

                        if nx >= 0 and nx < N and ny >= 0 and ny < M and frontier_cells[nx, ny] == 1:
                            q.append((nx, ny))
                label_id += 1
        # print(cluster_sizes)
        
        valid_cluster_ids = []
        for cluster_id in cluster_sizes:
            if self.is_valid_frontier_cluster(cluster_id, cluster_sizes):
                valid_cluster_ids.append(cluster_id)
        
        # plt.imshow(np.rot90(cluster_labels))
        # plt.show()

        # mask = np.zeros_like(cluster_labels)

        # for i, valid_id in enumerate(valid_cluster_ids):
        #     id_mask = cluster_labels == valid_id
        #     mask[id_mask] = i
        
        # plt.imshow(np.rot90(mask))
        # plt.show()

        # Compute Cluster Centers 
        cluster_centers = []
        for i, valid_id in enumerate(valid_cluster_ids):
            xs, ys = np.where(cluster_labels == valid_id)
            x_center = np.mean(xs).astype(int)
            y_center = np.mean(ys).astype(int)
            cluster_centers.append((x_center, y_center, valid_id))
        cluster_centers = np.array(cluster_centers)

        self.visualize_points(plt.gca())
        frontier_points_world_coords = self.batch_grid_to_approx_world_coords(cluster_centers[:, :2])
        plt.scatter(frontier_points_world_coords[:, 0], frontier_points_world_coords[:, 1])
        plt.show()

        
        # Filter Frontiers By How Far They Are From Obstacle Points
        obstacle_points = self.get_points()
        kd_tree = KDTree(data=obstacle_points)

        # Find the Single Nearest Neighbor
        dists, idxs = kd_tree.query(frontier_points_world_coords, k=1)
        

        min_dist_threshold = 400 #mm
        valid_frontier_mask = dists.flatten() > min_dist_threshold

        frontier_points_world_coords = frontier_points_world_coords[valid_frontier_mask]

        self.visualize_points(plt.gca())
        plt.scatter(frontier_points_world_coords[:, 0], frontier_points_world_coords[:, 1])
        plt.show()

        return frontier_points_world_coords
    
    def adjust_resolution(self, resolution=100):
        new_map = AdvancedMap(resolution=resolution)
        free_layer_coords_and_values, occupied_layer_coords_and_values = self.get_points_and_values_by_layer()

        # Populate New Map
        new_free_layer_coords = new_map.batch_world_to_grid_coords(free_layer_coords_and_values[:, :2])
        new_occupied_layer_coords = new_map.batch_world_to_grid_coords(occupied_layer_coords_and_values[:, :2])

        new_map.map[new_free_layer_coords[:, 0], new_free_layer_coords[:, 1], FREE] = free_layer_coords_and_values[:, 2]
        new_map.map[new_occupied_layer_coords[:, 0], new_occupied_layer_coords[:, 1], OCCUPIED] = occupied_layer_coords_and_values[:, 2] * 100

        return new_map
    
    def visualize(self, ax):
        probability_map = self.map_to_probability_map()
        ax.imshow((np.rot90(probability_map)))
    
    def visualize_layers(self, ax):
        ax[0].imshow(np.rot90(self.map[:, :, FREE]))
        ax[1].imshow(np.rot90(self.map[:, :, OCCUPIED]))
        
if __name__ == '__main__':
    from test_utils import load_saved_advanced_map
    advanced_map: AdvancedMap = load_saved_advanced_map(directory="saves/scenes/extensive_apartment")
    advanced_map.visualize(plt.gca())
    plt.show()

    frontier_candidates = advanced_map.get_frontier_candidates()
    plt.imshow(np.rot90(frontier_candidates))
    plt.show()

    fig, ax = plt.subplots(1, 2)
    advanced_map.visualize(ax[0])
    ax[1].imshow(np.rot90(frontier_candidates))
    plt.show()

    import time
    st = time.time()
    advanced_map.get_frontiers()
    et = time.time()

    print(f"Time to get frontiers: {et-st}")


    
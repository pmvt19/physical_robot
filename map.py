import numpy as np
import matplotlib.pyplot as plt
from icp import run_icp
from scipy.signal import convolve2d
from shapely import Point
import pickle
import os


# map_size = (100, 100)
# res = 0.1

# om = np.zeros((int(map_size[0]/res), int(map_size[1]/res)))

# Robot Algorithm Outline

# Set of motion commands: U

# for u in U:
#   motion = robot.move(u)
#   predicted_state = robot.predict_state(motion)
#   lidar_reading = robot.read_lidar()
#   updated_state = map.localize_robot(predicted_state, lidar_reading)
#   final_state = ?? (Some weighted average of predicted vs updated_state?)


# Map algorithm outline 
#  --- Ideally, we reuse the occupancy_map in pmpl
#  --- Or, rewrite that map entirely and replace the existing implementation
#  --- The current map does not support the robot being rotated

# map.localize_robot(predicted_state, lidar_reading):
#   T = icp(target=map_point_cloud, source=lidar_reading)
#   extracted_state = extract_state(T)
#   -- IMPORTANT --
#   add_points_to_map(lidar_reading) # Need to expand the map if possible
#   return extracted_state




"""
This map interface should be similar to the occupancy grid in the motion planning codebase. 

There should be a more advanced map that will compute the probability of free space. This will be 
implemented later.
"""
class Map():
    def __init__(self, resolution=10.0, origin=(45, 92)):
        self.resolution = resolution

        # map size
        # map_size_literal = np.array([12.0, 12.0]) # TODO: Figure out how to compute this value
        # map_size_literal = np.array([12000.0, 12000.0]) # TODO: Figure out how to compute this value
        map_size_literal = np.array([20000.0, 20000.0]) # TODO: Figure out how to compute this value
        map_size_discretized = (map_size_literal // self.resolution).astype(np.int32)
        print(f"Map Size Discritized Size: {map_size_discretized}")
        self.map = np.zeros(map_size_discretized)

        self.mx = self.map.shape[0] // 2
        self.my = self.map.shape[1] // 2

        print(f"Origin: {(self.mx, self.my)}")

        self.map_type_name = 'map'

        # if origin:
        #     self.mx = origin[0]
        #     self.my = origin[1]

        # self.update_map(initial_scan)
    # TODO: Fix in run_interactive_robot.py, run_semantic_slam.py, test_utils.py, simulation.py (archive)
    def init_map(self, initial_scan):
        self.update_map(initial_scan)

    def world_to_grid_coords(self, coords):
        ## Division Needed
        x, y = coords
        x = int(x / self.resolution)
        y = int(y / self.resolution)
        new_x = x + self.mx
        new_y = y + self.my

        return np.array([new_x, new_y])

    def batch_world_to_grid_coords(self, coords):
        grid_coords = np.copy(coords)
        grid_coords = grid_coords / self.resolution
        grid_coords = grid_coords.astype(np.int32)
        grid_coords[:, 0] += self.mx
        grid_coords[:, 1] += self.my
        return grid_coords

    def grid_to_approx_world_coords(self, coords):
        gx, gy = coords
        
        new_x = (gx - self.mx) * self.resolution
        new_y = (gy - self.my) * self.resolution

        return np.array([new_x, new_y])
    
    def batch_grid_to_approx_world_coords(self, coords):
        world_coords = np.copy(coords)
        world_coords[:, 0] -= self.mx
        world_coords[:, 1] -= self.my
        world_coords = world_coords * self.resolution
        return world_coords

    def update(self, scan, predicted_state):
        T = run_icp(scan, self.get_points(), predicted_state, visualize=False)
        updated_theta = np.arctan2(T[1,0],T[0,0]) % (2*np.pi)

        updated_x = T[0, 2]
        updated_y = T[1, 2]

        aligned_scan = (T@scan.T).T

        self.update_map(aligned_scan)
        return np.array([updated_x, updated_y, updated_theta])
    
    def set_known_clear(self, aligned_scan, updated_state):
        x, y = updated_state
        s_state = np.array([x, y]).reshape(-1, 1)
        repeated_state = np.repeat(s_state, len(aligned_scan), axis=1)
        line_segments = np.hstack((repeated_state, aligned_scan))
        dist_mask = self.compute_close_cells(line_segments)
        self.map[dist_mask] = 0.0

    def update_map(self, aligned_scan, updated_state=None):
        idxes = self.batch_world_to_grid_coords(aligned_scan)
        is_valid = self.validate_map_boundaries(idxes)
        # TODO: Clean this up if correct
        if is_valid:
            self.map[idxes[:, 0], idxes[:, 1]] = 1
        else:
            self.expand_map(idxes)
            idxes = self.batch_world_to_grid_coords(aligned_scan)
            self.map[idxes[:, 0], idxes[:, 1]] = 1
            
    def get_map_2d(self):
        return self.map
    
    def get_points(self):
        idxes = np.where(self.map == 1)
        xs, ys = idxes
        pc_idxes = np.stack((xs, ys), axis=1)
        pc_coords = self.batch_grid_to_approx_world_coords(pc_idxes)
        return pc_coords

    def get_points_and_values(self):
        threshold = 0.5 # TODO: Figure out what this value should be
        idxes = np.where(self.map > threshold)
        xs, ys = idxes
        pc_idxes = np.stack((xs, ys), axis=1)
        pc_coords = self.batch_grid_to_approx_world_coords(pc_idxes)
        pc_values = self.map[xs, ys].reshape(-1, 1)
        pc_coords_and_values = np.hstack((pc_coords, pc_values))
        return pc_coords_and_values
    
    def inflate_obstacles(self, kernel_size=3):
        kernel = np.ones((kernel_size, kernel_size))
        kernel[kernel_size//2, kernel_size//2] = 0

        # Convolve: counts neighbors of "1"s
        neighbor_mask = convolve2d((self.map == 1).astype(int), kernel, mode="same", boundary="fill", fillvalue=0)

        # Where neighbor_mask > 0 (adjacent to a 1) and current value != 1
        self.map[(neighbor_mask > 0) & (self.map != 1)] = 0.7

    def validate_map_boundaries(self, grid_coords):
        min_x = np.min(grid_coords[:, 0])
        max_x = np.max(grid_coords[:, 0])

        min_y = np.min(grid_coords[:, 1])
        max_y = np.max(grid_coords[:, 1])

        N, M, *_ = self.map.shape

        is_valid = (min_x >= 0 and max_x < N and min_y >= 0 and max_y < M) # Check if grid_coords are within bounds
        return is_valid
    
    def _compute_new_map_size(self, grid_coords):
        # TODO: New Map Size Should be computed based off world coordinates, not grid coordinates

        grid_coords = grid_coords[:, :2]
        mins = np.min(grid_coords, axis=0)
        maxs = np.max(grid_coords, axis=0)

        # Calculate which has the greatest diffs to the boundary

        # Mins
        min_diffs = 0 - mins

        # Maxs
        max_diffs = maxs - np.array(self.map.shape)

        all_diffs = np.concatenate((min_diffs, max_diffs), axis=0)
        all_diffs[all_diffs < 0] = 0 # Zero out non important diffs
        max_idx_diff = np.max(all_diffs)

        expansion_buffer = 0.3 # Increasing buffer size to prevent map expansion from being too small
        buffered_max_idx_diff = int((max_idx_diff * (1 + expansion_buffer)) + 0.5) # 0.5 is for Rounding
        
        N, M, *_ = self.map.shape
        new_map_size_discretized = np.array([N + 2*buffered_max_idx_diff, M + 2*buffered_max_idx_diff]) # In Idx Coords
        return new_map_size_discretized

    def expand_map(self, req_grid_coords):
        # approx_world_coords = self.get_points() # TODO: Decide whether these should include the value at that location
        approx_world_coords_and_values = self.get_points_and_values() # TODO: Decide whether these should include the value at that location
        approx_world_coords = approx_world_coords_and_values[:, :2]
        values = approx_world_coords_and_values[:, 2]

        ### --- Expand Map Here --- ###

        map_size_discretized = self._compute_new_map_size(grid_coords=req_grid_coords)
        map_size_discretized = map_size_discretized.astype(np.int32)
        print(f"Expanded Map Discritized Size: {map_size_discretized}")
        self.map = np.zeros(map_size_discretized)
        self.mx = self.map.shape[0] // 2
        self.my = self.map.shape[1] // 2

        ### --- Expand Map Here --- ###
        new_grid_coords = self.batch_world_to_grid_coords(approx_world_coords)
        self.map[new_grid_coords[:, 0], new_grid_coords[:, 1]] = values # TODO: Linked with previous todo in this function^

    def draw_state(self, ax, state):
        x, y, theta = state
        x, y = x / self.resolution + self.mx, y / self.resolution + self.my
        robot_radius = (227 / 2) * 0.9
        robot_outline = Point([x, y]).buffer(robot_radius/self.resolution)
        ax.fill(*robot_outline.exterior.xy, color='blue', alpha=0.3)
    
    def visualize(self, ax):
        ax.imshow((np.rot90(self.map[::, ::]))*255)
    
    def visualize_points(self, ax):
        points = self.get_points()
        ax.scatter(points[:, 0], points[:, 1])

    def save(self, map_save_dir, file_name_ext="final"):
        save_path = os.path.join(map_save_dir, self.map_type_name)
        os.makedirs(save_path, exist_ok=True)
        pickle.dump(self, open(f"{save_path}/{self.map_type_name}_object_{file_name_ext}.pickle", "wb"))

if __name__ == '__main__':
    points = np.load('data/new_slam_data/scene_1.npy')
    mymap = Map(initial_scan=points)

    # print(mymap.world_to_grid_coords((-511, -500)))
    # print(mymap.grid_to_approx_world_coords(mymap.world_to_grid_coords((-500, -500))))

    mymap.update_map(points)
    mymap.inflate_obstacles()
    mymap.visualize(ax=plt.gca())
    mymap.draw_state(plt.gca(), [40.0, 0.0, 0.0])
    plt.show()
    
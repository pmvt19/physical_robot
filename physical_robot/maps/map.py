import numpy as np
import matplotlib.pyplot as plt
from physical_robot.algorithms.icp import run_icp
from scipy.signal import convolve2d
from shapely import Point
import pickle
import os
from physical_robot.utils import create_circular_kernel


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
    map_type_name = "map"

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

        # self.map_type_name = 'map'

        self.needs_inflation_update = True # TODO: Deprecate this field
        self.current_inflation_radius = 0

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

    def get_map_range(self):
        low_range = self.grid_to_approx_world_coords([0.0, 0.0])
        high_range = self.grid_to_approx_world_coords([*self.get_shape_2d()])
        x_range = [low_range[0], high_range[0]]
        y_range = [low_range[1], high_range[1]]
        return x_range, y_range
    
    def get_value_at_grid_coords(self, coords):
        gx, gy = coords
        map_2d = self.get_map_2d()
        return map_2d[gx, gy]
    
    def batch_get_value_at_grid_coords(self, coords):
        map_2d = self.get_map_2d()
        return map_2d[coords[:, 0], coords[:, 1]]
    
    def get_shape_2d(self):
        return self.map.shape

    def update(self, scan, predicted_state):
        T = run_icp(scan, self.get_points(), predicted_state, visualize=False)
        updated_theta = np.arctan2(T[1,0],T[0,0]) % (2*np.pi)

        updated_x = T[0, 2]
        updated_y = T[1, 2]

        aligned_scan = (T@scan.T).T

        self.update_map(aligned_scan)
        return np.array([updated_x, updated_y, updated_theta])

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
        self.needs_inflation_update = True
        self.current_inflation_radius = 0
            
    def get_map_2d(self):
        return self.map
    
    def get_points(self):
        idxes = np.where(self.map == 1)
        xs, ys = idxes
        pc_idxes = np.stack((xs, ys), axis=1)
        pc_coords = self.batch_grid_to_approx_world_coords(pc_idxes)
        return pc_coords

    def get_points_and_values(self, threshold=0.5):
        idxes = np.where(self.map > threshold)
        xs, ys = idxes
        pc_idxes = np.stack((xs, ys), axis=1)
        pc_coords = self.batch_grid_to_approx_world_coords(pc_idxes)
        pc_values = self.map[xs, ys].reshape(-1, 1)
        pc_coords_and_values = np.hstack((pc_coords, pc_values))
        return pc_coords_and_values
    
    def _inflate_obstacles(self, inflation_radius=210):
        self.inflated_map = self.map.copy()
        kernel_size = int(inflation_radius // self.resolution) # Hard Coded to the radius of the robot

        kernel = create_circular_kernel(kernel_size, kernel_size/2)

        # Convolve: counts neighbors of "1"s
        neighbor_mask = convolve2d((self.map == 1).astype(int), kernel, mode="same", boundary="fill", fillvalue=0)

        # Where neighbor_mask > 0 (adjacent to a 1) and current value != 1
        self.inflated_map[(neighbor_mask > 0) & (self.map != 1)] = 1
        self.needs_inflation_update = False
        self.current_inflation_radius = inflation_radius
    
    def get_inflated_map_2d(self, inflation_radius=210):
        assert(inflation_radius > 0)
        if self.current_inflation_radius != inflation_radius:
            self._inflate_obstacles(inflation_radius=inflation_radius)
        return self.inflated_map

    def get_frontiers(self):
        raise NotImplementedError

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
        max_diffs = maxs - np.array(self.get_shape_2d())

        all_diffs = np.concatenate((min_diffs, max_diffs), axis=0)
        all_diffs[all_diffs < 0] = 0 # Zero out non important diffs
        max_idx_diff = np.max(all_diffs)

        expansion_buffer = 0.3 # Increasing buffer size to prevent map expansion from being too small
        buffered_max_idx_diff = int((max_idx_diff * (1 + expansion_buffer)) + 0.5) # 0.5 is for Rounding
        
        N, M = self.get_shape_2d()
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
        self.needs_inflation_update = True
        self.current_inflation_radius = 0

    def adjust_resolution(self, resolution=100.0):
        new_map = Map(resolution=resolution)
        points_and_values = self.get_points_and_values()
        points = points_and_values[:, :2]
        values = points_and_values[:, 2]

        # Populate New Map
        new_grid_coords = new_map.batch_world_to_grid_coords(points)
        new_map.map[new_grid_coords[:, 0], new_grid_coords[:, 1]] = values

        return new_map

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
    
    ### --- Saving Map Functions --- ###
    @classmethod
    def get_save_dir(cls, map_save_dir):
        save_path = os.path.join(map_save_dir, cls.map_type_name)
        os.makedirs(save_path, exist_ok=True)
        return save_path
    
    def save_raw_map(self, map_save_dir, file_name_ext="final"):
        pickle.dump(self.map, open(f"{self.get_save_dir(map_save_dir)}/{self.map_type_name}_raw_map_{file_name_ext}.pickle", "wb"))

    def save_map_2d(self, map_save_dir, file_name_ext="final"):
        pickle.dump(self.get_map_2d(), open(f"{self.get_save_dir(map_save_dir)}/{self.map_type_name}_map_2d_{file_name_ext}.pickle", "wb"))

    def save(self, map_save_dir, file_name_ext="final"):
        pickle.dump(self, open(f"{self.get_save_dir(map_save_dir)}/{self.map_type_name}_object_{file_name_ext}.pickle", "wb"))

    ### --- Load Map Function --- ###
    @classmethod
    def load_map(cls, map_save_dir, file_name_ext="final"):
        loaded_map = pickle.load(open(f"{cls.get_save_dir(map_save_dir)}/{cls.map_type_name}_object_{file_name_ext}.pickle", "rb"))
        return loaded_map

if __name__ == '__main__':
    mymap = Map.load_map(map_save_dir="saves/scenes/extensive_apartment")
    mymap._inflate_obstacles() # DO NOT CALL LIKE THIS
    mymap.visualize(ax=plt.gca())
    mymap.draw_state(plt.gca(), [40.0, 0.0, 0.0])
    plt.show()
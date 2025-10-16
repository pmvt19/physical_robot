import numpy as np
import matplotlib.pyplot as plt
from icp import run_icp
from scipy.signal import convolve2d
from shapely import Point

from robot import Robot


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
    def __init__(self, initial_scan, resolution=10.0, origin=(45, 92)):
        self.resolution = resolution

        # map size
        map_size_literal = np.array([12000.0, 12000.0]) # TODO: Figure out how to compute this value
        map_size_discretized = (map_size_literal // self.resolution).astype(np.int32)
        print(f"Map Size Discritized Size: {map_size_discretized}")
        self.map = np.zeros(map_size_discretized)

        self.mx = self.map.shape[0] // 2
        self.my = self.map.shape[1] // 2

        print(f"Origin: {(self.mx, self.my)}")

        # if origin:
        #     self.mx = origin[0]
        #     self.my = origin[1]

        self.update_map(initial_scan)

        ## --- Computing Grid Centers --- ##
        x_idxes = np.arange(self.map.shape[0])
        y_idxes = np.arange(self.map.shape[1])

        xs, ys = np.meshgrid(x_idxes, y_idxes)
        all_idxes = np.stack((xs.flatten(), ys.flatten()), axis=1)
        self.grid_centers = self.batch_grid_to_approx_world_coords(all_idxes) + (self.resolution / 2)
        ## --- Computing Grid Centers --- ##
    
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
        # updated_theta = np.arccos(T[0, 0])
        updated_theta = np.arctan2(T[1,0],T[0,0]) % (2*np.pi)

        updated_x = T[0, 2]
        updated_y = T[1, 2]

        aligned_scan = (T@scan.T).T

        self.update_map(aligned_scan)
        return np.array([updated_x, updated_y, updated_theta])
    
    def compute_close_cells(line_segments):
        # Batch to Line Segments dists
        # dists = line_segs_to_points_dists(line_segments, self.grid_centers (flattened...)) # Use Gemini to get this
        # dists.reshape # Need make sure we are reshaping correctly 
        # - Mainly, not sure if meshgrid reshaping will order the points correctly
        # mask = dists < (resolution * math.sqrt(2))
        # return mask??

        # Return mask of what cells need to be updated
        raise NotImplementedError
    
    def set_known_clear(self, aligned_scan, updated_state):
        x, y = updated_state
        s_state = np.array([x, y]).reshape(-1, 1)
        repeated_state = np.repeat(s_state, len(aligned_scan), axis=1)
        line_segments = np.hstack((repeated_state, aligned_scan))
        dist_mask = self.compute_close_cells(line_segments)
        self.map[dist_mask] = 0.0

    def update_map(self, aligned_scan):
        idxes = self.batch_world_to_grid_coords(aligned_scan)
        self.map[idxes[:, 0], idxes[:, 1]] = 1
    
    def get_points(self):
        idxes = np.where(self.map == 1)
        xs, ys = idxes
        pc_idxes = np.stack((xs, ys), axis=1)
        pc_coords = self.batch_grid_to_approx_world_coords(pc_idxes)
        return pc_coords
    
    def inflate_obstacles(self, kernel_size=3):
        kernel = np.ones((kernel_size, kernel_size))
        kernel[kernel_size//2, kernel_size//2] = 0

        # Convolve: counts neighbors of "1"s
        neighbor_mask = convolve2d((self.map == 1).astype(int), kernel, mode="same", boundary="fill", fillvalue=0)

        # Where neighbor_mask > 0 (adjacent to a 1) and current value != 1
        self.map[(neighbor_mask > 0) & (self.map != 1)] = 0.7

    def draw_state(self, ax, state):
        x, y, theta = state
        x, y = x / self.resolution + self.mx, y / self.resolution + self.my
        robot_radius = (227 / 2) * 0.9
        robot_outline = Point([x, y]).buffer(robot_radius/self.resolution)
        ax.fill(*robot_outline.exterior.xy, color='blue', alpha=0.3)

    def expand_map(self):
        raise NotImplementedError
    
    def visualize(self, ax):
        ax.imshow((np.rot90(self.map[::, ::]))*255)


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
    
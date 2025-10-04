import numpy as np

map_size = (100, 100)
res = 0.1

om = np.zeros((int(map_size[0]/res), int(map_size[1]/res)))

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
    def __init__(self):
        # self.resolution = 1.0 #mm
        self.resolution = 10.0 #mm

        # map size
        map_size_literal = np.array([7000.0, 7000.0]) # [-3499, 3499]
        map_size_discretized = (map_size_literal // self.resolution).astype(np.int32)
        print(f"Map Size Discritized Size: {map_size_discretized}")
        self.map = np.zeros(map_size_discretized)

        self.mx = self.map.shape[0] // 2
        self.my = self.map.shape[1] // 2

        print(f"Origin: {(self.mx, self.my)}")

    def int_points_to_idxes(self, points):
        """
        Scales points to idxes of the map
        """
        # TODO: Implement this
        return points # (N, 2)
    
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
        ## Division Needed
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
        pass
    
    def update_map(self, aligned_scan):
        aligned_scan_int = aligned_scan.astype(np.int32)
        idxes = self.int_points_to_idxes(aligned_scan_int)
        self.map[idxes[:, 0], idxes[:, 1]] = 1
    
    def get_points(self):
        # Should scale
        idxes = np.where(self.map == 1)
        # TODO: Fix the returnable result
        return idxes
    
    def visualize(self, ax):
        ax.imshow(self.map)


if __name__ == '__main__':
    mymap = Map()

    # print(mymap.world_to_grid_coords((-511, -500)))
    print(mymap.grid_to_approx_world_coords(mymap.world_to_grid_coords((-500, -500))))
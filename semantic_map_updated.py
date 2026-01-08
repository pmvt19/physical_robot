import numpy as np
import matplotlib.pyplot as plt
import time
from heapq import *
from map import Map
from utils import timer
from sklearn.neighbors import KDTree
from icp import run_icp
from skimage.segmentation import expand_labels
import pickle

class MapInterface(): # Renamed to not override actual Map implementation
    def __init__(self, resolution=10.0, origin=(45, 92)):
        pass

    def init_map(self, initial_scan):
        pass

    def world_to_grid_coords(self, coords):
        pass

    def batch_world_to_grid_coords(self, coords):
        pass

    def grid_to_approx_world_coords(self, coords):
        pass
    
    def batch_grid_to_approx_world_coords(self, coords):
        pass

    def update(self, scan, predicted_state):
       pass

    def update_map(self, aligned_scan, updated_state=None):
        pass
            
    def get_map_2d(self):
        pass
    
    def get_points(self):
        pass

    def get_points_and_values(self):
       pass
    
    def inflate_obstacles(self, kernel_size=3):
        pass

    def validate_map_boundaries(self, grid_coords):
        pass
    
    def _compute_new_map_size(self, grid_coords):
        pass

    def expand_map(self, req_grid_coords):
        pass

    def draw_state(self, ax, state):
       pass
    
    def visualize(self, ax):
        pass
    
    def visualize_points(self, ax):
        pass
    
    def get_save_dir(self, map_save_dir):
        pass

    def save(self, map_save_dir, file_name_ext="final"):
       pass

class SemanticMap(Map):

    def __init__(self, map_obj: Map):
        self.geometric_map: Map = map_obj
        self.semantic_layer = np.zeros((self.map.map.shape[0], self.map.map.shape[1], 2)) # axis 2: 0 -> Room Level Information, 1 -> Object Level Information
        self.flood_filled_map = None
        self.map_type_name = 'semantic_map'
        self.invalid_objects = ['wall', 'floor', 'ceiling', 'door']

        self.layer_name_to_idx = {
            'room' : 0,
            'object' : 1
        }

        self.room_to_id = {
            'none-reserved' : 0
        }

        self.object_to_id = {
            'none-reserved' : 0
        }
    
    def init_map(self, scan):
        self.geometric_map.init_map(scan)
    
    def world_to_grid_coords(self, coords):
        return self.geometric_map.world_to_grid_coords(coords)

    def batch_world_to_grid_coords(self, coords):
        return self.batch_grid_to_approx_world_coords(coords)
    
    def get_room_id(self, room):
        if room in self.room_to_id:
            return self.room_to_id[room]
        else:
            next_id = len(self.room_to_id)
            self.room_to_id[room] = next_id
            return self.room_to_id[room]
    
    # TODO Improve the logic here
    def get_object_id(self, object):
        if object in self.object_to_id:
            return self.object_to_id[object]
        else:
            next_id = len(self.object_to_id)
            self.object_to_id[object] = next_id
            return self.object_to_id[object]

    def grid_to_approx_world_coords(self, coords):
        return self.geometric_map.grid_to_approx_world_coords(coords)
    
    def batch_grid_to_approx_world_coords(self, coords):
        self.batch_grid_to_approx_world_coords(coords)

    def update(self, scan, predicted_state):
       return self.geometric_map.update(scan, predicted_state)

    def update_map(self, aligned_scan, updated_state=None):
        self.geometric_map.update_map(aligned_scan, updated_state)

    def get_map_2d(self):
        return self.geometric_map.get_map_2d()
    
    def get_points(self):
        return self.get_points()
    
    def get_points_and_values(self, threshold=0.5):
        return self.geometric_map.get_points_and_values(threshold=threshold)
    
    def inflate_obstacles(self, kernel_size=3):
        self.geometric_map.inflate_obstacles(kernel_size=kernel_size)

    def validate_map_boundaries(self, grid_coords):
        self.geometric_map.validate_map_boundaries(grid_coords)

    def _compute_new_map_size(self, grid_coords):
        raise NotImplementedError
        self.geometric_map._compute_new_map_size(grid_coords)
    
    def expand_map(self, req_grid_coords):
        print("Semantic Map does not support expansions yet")
        raise NotImplementedError
        self.geometric_map.expand_map(req_grid_coords)
    
    def draw_state(self, ax, state):
        raise NotImplementedError
    
    # Design How to Visualize Later???
    def visualize(self, ax):
        # raise NotImplementedError
        self.geometric_map.visualize(ax)
    
    def visualize_semantics(self, ax):
        raise NotImplementedError
    
    def visualize_points(self, ax):
        raise NotImplementedError
    
    def get_save_dir(self, map_save_dir):
        raise NotImplementedError
    
    # def visualize(self, ax, visualize_layers=False, visualize_flood_fills=False):
    #     # TODO: Add logic to only show geometric map if ax is not a list of axes
    #     self.map.visualize(ax[0])

    #     if visualize_layers:
    #         semantic_room_layer = self.semantic_layer[:, :, self.layer_name_to_idx['room']]
    #         semantic_object_layer = self.semantic_layer[:, :, self.layer_name_to_idx['object']]

    #         # ax[1].set_title("Room Layer Semantics")
    #         ax[1].imshow(np.rot90(semantic_room_layer))

    #         # ax[2].set_title("Object Layer Semantics")
    #         ax[2].imshow(np.rot90(semantic_object_layer))

    #     if visualize_layers and visualize_flood_fills:
    #         # ax[3].set_title("Room Layer Semantics - Flood Fill")
    #         ax[3].imshow(np.rot90(self.flood_filled_map[:, :, self.layer_name_to_idx['room']]))

    #         # ax[4].set_title("Object Layer Semantics - Flood Fill")
    #         ax[4].imshow(np.rot90(self.flood_filled_map[:, :, self.layer_name_to_idx['object']]))
    
    #     if not visualize_layers and visualize_flood_fills:
    #         print("Cannot visualize only flood fills")
    
    # def save(self, map_save_dir) # Not Required? Handled by Polymorphism??
    def label_point_cloud(self, pc, segmented_img):
        pass

    # Rename to semantic_update?
    def update_geometry_and_semantics(self, lidar_coords, pc_flattened_coords_and_labels, predicted_state, option=False):
        T = run_icp(lidar_coords, self.geometric_map.get_points(), predicted_state, visualize=False)
        updated_theta = np.arctan2(T[1,0],T[0,0]) % (2*np.pi)
        updated_x = T[0, 2]
        updated_y = T[1, 2]
        updated_state = np.array([updated_x, updated_y, updated_theta])

        aligned_lidar_coords = (T@lidar_coords.T).T

        # Remove Labels for Transformation
        pc_flattened_coords = pc_flattened_coords_and_labels[:, :2]

        # Make Homogeneous Coordinates
        homogeneous_pc_flattened_coords = np.hstack((pc_flattened_coords, np.ones((len(pc_flattened_coords), 1))))

        aligned_pc_flattened_coords = (T@homogeneous_pc_flattened_coords.T).T
        aligned_pc_flattened_coords = aligned_pc_flattened_coords[:, :2] # Remove Homogeneous Coordinates

        aligned_pc_flattened_coords_and_labels = np.concatenate((aligned_pc_flattened_coords, pc_flattened_coords_and_labels[:, 2:]), axis=1)

        self.update_geometry_and_semantics_map(aligned_lidar_coords, aligned_pc_flattened_coords_and_labels, updated_state)
        return updated_state
    
    def update_geometry_and_semantics_map(self, aligned_scan, semantics, updated_state):
        # Update the geometric map
        self.update_map(aligned_scan, updated_state)

        # Update the semantic map
        self.update_semantic_map(semantics)

    def update_semantic_map(self, semantics):
        """
        semantics: (N, 4) matrix where axis {0,1} is the point in 2d world coords and axis {2, 3} are labels??
        """
        semantic_grid_coords = self.batch_world_to_grid_coords(semantics[:, :2])
        N, M = self.get_map_2d().shape
        valid_mask = np.logical_and.reduce((
            semantic_grid_coords[:, 0] >= 0,
            semantic_grid_coords[:, 0] < N,
            semantic_grid_coords[:, 1] >= 0,
            semantic_grid_coords[:, 1] < M
        ))
        print("valid points:", valid_mask.sum(), "out of", semantic_grid_coords.shape[0])
        semantic_grid_coords = semantic_grid_coords[valid_mask]
        semantic_info = semantics[:, 2:][valid_mask]
        self.semantic_layer[semantic_grid_coords[:, 0], semantic_grid_coords[:, 1], :] = semantic_info
    
    def update_semantic_map_single_layer(self, semantics, layer):
        """
        semantics: (N, 3) matrix where axis {0,1} is the point in 2d world coords and axis {2} is labels
        """
        semantic_grid_coords = self.batch_world_to_grid_coords(semantics[:, :2])
        N, M = self.get_map_2d().shape
        valid_mask = np.logical_and.reduce((
            semantic_grid_coords[:, 0] >= 0,
            semantic_grid_coords[:, 0] < N,
            semantic_grid_coords[:, 1] >= 0,
            semantic_grid_coords[:, 1] < M
        ))
        print("valid points:", valid_mask.sum(), "out of", semantic_grid_coords.shape[0])
        semantic_grid_coords = semantic_grid_coords[valid_mask]
        semantic_info = semantics[:, 2][valid_mask]
        self.semantic_layer[semantic_grid_coords[:, 0], semantic_grid_coords[:, 1], self.layer_name_to_idx[layer]] = semantic_info
    
    def inflate_semantics(self, distance=10):
        self.semantic_layer[:, :, 0] = expand_labels(self.semantic_layer[:, :, 0], distance=distance)
        self.semantic_layer[:, :, 1] = expand_labels(self.semantic_layer[:, :, 1], distance=distance)
    
    ## -- FLOOD FILLING MAP FUNCTIONS -- ##

    @timer
    def flood_fill(self, limit_fill_extent=False, method='bfs'):
        if method == 'bfs':
            self._bfs_flood_fill(limit_fill_extent)
        elif method == 'nearest_neighbor':
            self._nearest_neighbor_flood_fill(limit_fill_extent)
        else:
            raise ValueError(f"Unknown flood fill method: {method}")
    
    def _bfs_flood_fill(self, limit_fill_extent=False):
        # Queue with starting seed for classes
        # BFS and labeling classes with closest labels
        # IDEA: Implement this function in C++ and use python bindings here?
        # raise NotImplementedError

        map_points = self.map.get_points()
        grid_coords = self.map.batch_world_to_grid_coords(map_points)
        q = []
        for x, y in grid_coords:
            label = self.semantic_layer[x, y]
            if label[0] > 0:
                q.append((x,y,label))

        self.flood_filled_map = np.copy(self.semantic_layer)
        
        neighbors = [(0,-1),(0,1),(1,0),(-1,0)]
        # neighbors = [(0,-1),(0,1),(1,0),(-1,0),(1,1),(-1,-1),(-1,1),(1,-1)]
        # neighbors = [(1,1),(-1,-1),(-1,1),(1,-1)]
        # neighbors = [(-1,0),(0,-1),(0,1),(1,0)]
        visited = set()

        min_x, min_y = 0, 0
        max_x, max_y = self.map.map.shape

        if limit_fill_extent:
            min_x, min_y = np.min(grid_coords, axis=0)
            max_x, max_y = np.max(grid_coords, axis=0)

        while q:
            x, y, label = q.pop(0)
            if (x, y) in visited:
                continue
            visited.add((x,y))

            self.flood_filled_map[x,y] = label
            
            for dx, dy in neighbors:
                nx = x + dx
                ny = y + dy
                if nx >= min_x and nx < max_x and ny >= min_y and ny < max_y and self.map.map[nx, ny] == 0 and (nx, ny) not in visited: #TODO: Check why this improves speed
                    q.append((nx, ny, label))
    
    def _nearest_neighbor_flood_fill(self, limit_fill_extent=False):
        map_points = self.map.get_points()
        grid_coords = self.map.batch_world_to_grid_coords(map_points)
        min_grid_coords = np.min(grid_coords, axis=0)
        max_grid_coords = np.max(grid_coords, axis=0)
        kd_tree = KDTree(grid_coords)
        semantic_labels = self.semantic_layer[grid_coords[:, 0], grid_coords[:, 1]]
        grid_xs, grid_ys = np.where(self.map.get_map_2d() < 0.5)
        grid_coords = np.stack((grid_xs, grid_ys), axis=1)
        if limit_fill_extent:
            semantic_labeling_mask_xs = np.logical_and(grid_coords[:, 0] >= min_grid_coords[0], grid_coords[:, 0] <= max_grid_coords[0])
            semantic_labeling_mask_ys = np.logical_and(grid_coords[:, 1] >= min_grid_coords[1], grid_coords[:, 1] <= max_grid_coords[1])
            semantic_labeling_mask = np.logical_and(semantic_labeling_mask_xs, semantic_labeling_mask_ys)
            grid_coords = grid_coords[semantic_labeling_mask]
        _, idxes = kd_tree.query(grid_coords, k=1)
        self.flood_filled_map = np.copy(self.semantic_layer)
        self.flood_filled_map[grid_coords[:, 0], grid_coords[:, 1]] = semantic_labels[idxes[:, 0]]

if __name__ == '__main__':
    from test_utils import load_saved_map, load_saved_semantic_map
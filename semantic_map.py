import numpy as np
import matplotlib.pyplot as plt
import time
from heapq import *
from map import Map
from utils import timer
from sklearn.neighbors import KDTree
from icp import run_icp
from skimage.segmentation import expand_labels

ROOM_LAYER = 0
OBJECT_LAYER = 1

class SemanticMap():
    def __init__(self, map_obj):
        self.map : Map = map_obj
        self.semantic_layer = np.zeros((self.map.map.shape[0], self.map.map.shape[1], 2)) # axis 2: 0 -> Room Level Information, 1 -> Object Level Information
        self.flood_filled_map = None

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

    def update(self, lidar_coords, pc_flattened_coords_and_labels, predicted_state, option=False):
        T = run_icp(lidar_coords, self.map.get_points(), predicted_state, visualize=False)
        updated_theta = np.arctan2(T[1,0],T[0,0]) % (2*np.pi)
        updated_x = T[0, 2]
        updated_y = T[1, 2]

        aligned_lidar_coords = (T@lidar_coords.T).T
        pc_flattened_coords = pc_flattened_coords_and_labels[:, :2]

        # Make Homogeneous Coordinates
        homogeneous_pc_flattened_coords = np.hstack((pc_flattened_coords, np.ones((len(pc_flattened_coords), 1))))

        aligned_pc_flattened_coords = (T@homogeneous_pc_flattened_coords.T).T
        aligned_pc_flattened_coords = aligned_pc_flattened_coords[:, :2] # Remove Homogeneous Coordinates

        aligned_pc_flattened_coords_and_labels = np.concatenate((aligned_pc_flattened_coords, pc_flattened_coords_and_labels[:, 2:]), axis=1)
        self.update_map(aligned_lidar_coords, aligned_pc_flattened_coords_and_labels)
        return np.array([updated_x, updated_y, updated_theta])

    def update_semantic_map(self, semantics):
        """
        semantics: (N, 4) matrix where axis {0,1} is the point in 2d world coords and axis {2, 3} are labels??
        """
        semantic_grid_coords = self.map.batch_world_to_grid_coords(semantics[:, :2])
        N, M = self.map.map.shape
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
        semantics: (N, 4) matrix where axis {0,1} is the point in 2d world coords and axis {2, 3} are labels??
        """
        semantic_grid_coords = self.map.batch_world_to_grid_coords(semantics[:, :2])
        N, M = self.map.map.shape
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

    def update_map(self, aligned_scan, semantics):
        # Update the geometric map
        self.map.update_map(aligned_scan)

        # Update the semantic map
        self.update_semantic_map(semantics)
    
    def map_update(self, scan, predicted_state): # Previously update
        return self.map.update(scan, predicted_state)
    
    def inflate_semantics(self, distance=10):
        self.semantic_layer[:, :, 0] = expand_labels(self.semantic_layer[:, :, 0], distance=distance)
        self.semantic_layer[:, :, 1] = expand_labels(self.semantic_layer[:, :, 1], distance=distance)

    def inflate_obstacles(self):
        self.map.inflate_obstacles()

    def expand_map(self):
        print("Semantic Map does not support expansions yet")
        raise NotImplementedError
    
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
        grid_xs, grid_ys = np.where(self.map.map == 0)
        grid_coords = np.stack((grid_xs, grid_ys), axis=1)
        if limit_fill_extent:
            semantic_labeling_mask_xs = np.logical_and(grid_coords[:, 0] >= min_grid_coords[0], grid_coords[:, 0] <= max_grid_coords[0])
            semantic_labeling_mask_ys = np.logical_and(grid_coords[:, 1] >= min_grid_coords[1], grid_coords[:, 1] <= max_grid_coords[1])
            semantic_labeling_mask = np.logical_and(semantic_labeling_mask_xs, semantic_labeling_mask_ys)
            grid_coords = grid_coords[semantic_labeling_mask]
        _, idxes = kd_tree.query(grid_coords, k=1)
        self.flood_filled_map = np.copy(self.semantic_layer)
        self.flood_filled_map[grid_coords[:, 0], grid_coords[:, 1]] = semantic_labels[idxes[:, 0]]
    
    def visualize(self, ax, visualize_layers=False, visualize_flood_fills=False):
        self.map.visualize(ax[0])

        if visualize_layers:
            semantic_room_layer = self.semantic_layer[:, :, self.layer_name_to_idx['room']]
            semantic_object_layer = self.semantic_layer[:, :, self.layer_name_to_idx['object']]

            # ax[1].set_title("Room Layer Semantics")
            ax[1].imshow(np.rot90(semantic_room_layer))

            # ax[2].set_title("Object Layer Semantics")
            ax[2].imshow(np.rot90(semantic_object_layer))

        if visualize_layers and visualize_flood_fills:
            # ax[3].set_title("Room Layer Semantics - Flood Fill")
            ax[3].imshow(np.rot90(self.flood_filled_map[self.layer_name_to_idx['room']]))

            # ax[4].set_title("Object Layer Semantics - Flood Fill")
            ax[4].imshow(np.rot90(self.flood_filled_map[self.layer_name_to_idx['object']]))
    
        if not visualize_layers and visualize_flood_fills:
            print("Cannot visualize only flood fills")


def pseudolabel_map(semantic_map : SemanticMap):
    # semantic_map.visualize(plt.gca())
    semantic_map.map.visualize_points(plt.gca())
    plt.show()

    # Inject Fake Semantic Labels
    map_points = semantic_map.map.get_points() # (9312, 2) for apartment labels
    # print(type(map_points), map_points.shape)
    # exit()

    label_values = np.zeros((map_points.shape[0],)).astype(np.int32)
    office_label_mask = np.logical_and(map_points[:, 0] < -1514, map_points[:, 1] > 1000)
    label_values[office_label_mask] = 1

    dining_room_label_mask = np.logical_and(map_points[:, 0] > -729, map_points[:, 1] > 809)
    label_values[dining_room_label_mask] = 2

    kitchen_label_mask = np.logical_and(map_points[:, 0] > 1249, map_points[:, 1] > -1200)
    label_values[kitchen_label_mask] = 3

    room_label_mask = np.logical_and(map_points[:, 0] < -655, map_points[:, 1] < -1241)
    label_values[room_label_mask] = 4

    entrance_label_mask = np.logical_and(map_points[:, 0] > 859, map_points[:, 1] < -4113)
    label_values[entrance_label_mask] = 5
    
    grid_coords = semantic_map.map.batch_world_to_grid_coords(map_points)

    semantic_map.semantic_layer[grid_coords[:, 0], grid_coords[:, 1], 0] = label_values
    plt.imshow(np.rot90(semantic_map.semantic_layer[:, :, 0]))
    plt.show()

if __name__ == '__main__':
    from test_utils import load_saved_map, load_saved_semantic_map

    # map_obj = load_saved_map(directory='./saves/scenes/apartment/map')
    # semantic_map = SemanticMap(map_obj=map_obj)
    semantic_map : SemanticMap = load_saved_semantic_map()

    # pseudolabel_map(semantic_map)
    # semantic_map.inflate_semantics(30)
    # semantic_map.flood_fill(limit_fill_extent=False, method='nearest_neighbor')
    semantic_map.flood_fill(limit_fill_extent=False, method='bfs')

    print(semantic_map.object_to_id)

    fig, ax = plt.subplots(1, 3)

    semantic_map.visualize(ax, layer='room')
    plt.show()
    # semantic_map.map.visualize_points(ax[1][0])

    # fig, ax = plt.subplots(2, 3)

    # semantic_map.visualize(ax[0], layer='room')

    # semantic_map.map.visualize_points(ax[1][0])
    # Imports only needed here for prm creation
    # from motion_planning.prm import PRM
    # from run_localize_and_plan import create_or_load_prm
    # from robot_space import PhysicalRobotSpace

    # robot = PhysicalRobotSpace(semantic_map.map)
    # prm : PRM = create_or_load_prm(scene='apartment', robot=robot) # TODO: Doesn't work with new env python 3.10

    # semantic_map.map.visualize_points(ax[1][1])
    # prm.draw(ax[1][1])
    # plt.show()



    # TEST
    # random_values = np.random.random(size=(1200*1200))
    # st = time.time()
    # np.sort(random_values)
    # et = time.time()
    # print(f"Time: {et - st}")
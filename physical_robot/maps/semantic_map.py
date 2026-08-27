import numpy as np
import matplotlib.pyplot as plt
import time
from physical_robot.maps import Map
from physical_robot.utils import timer
from sklearn.neighbors import KDTree
from physical_robot.algorithms.icp import run_icp
from skimage.segmentation import expand_labels
import pickle

class SemanticMap(Map):
    map_type_name = 'semantic_map'

    def __init__(self, map_obj: Map):
        self.geometric_map: Map = map_obj
        self.resolution = self.geometric_map.resolution
        self.semantic_layer = np.zeros((self.get_map_2d().shape[0], self.get_map_2d().shape[1], 2)) # axis 2: 0 -> Room Level Information, 1 -> Object Level Information
        self.flood_filled_map = None
        # self.map_type_name = 'semantic_map'

        self.invalid_rooms = set(['wall', 'room'])
        self.invalid_objects = set(['wall', 'floor', 'ceiling', 'door', 'window', 'person', 'sky'])

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
    
    ## -- GEOMETRIC MAP WRAPPER FUNCTIONS -- ##
    
    def init_map(self, initial_scan):
        self.geometric_map.init_map(initial_scan)
    
    def world_to_grid_coords(self, coords):
        return self.geometric_map.world_to_grid_coords(coords)

    def batch_world_to_grid_coords(self, coords):
        return self.geometric_map.batch_world_to_grid_coords(coords)

    def grid_to_approx_world_coords(self, coords):
        return self.geometric_map.grid_to_approx_world_coords(coords)
    
    def batch_grid_to_approx_world_coords(self, coords):
        return self.geometric_map.batch_grid_to_approx_world_coords(coords)

    def get_shape_2d(self):
        return self.geometric_map.get_shape_2d()

    def update(self, scan, predicted_state):
       return self.geometric_map.update(scan, predicted_state)

    def update_map(self, aligned_scan, updated_state=None):
        self.geometric_map.update_map(aligned_scan, updated_state)

    def get_map_2d(self):
        return self.geometric_map.get_map_2d()
    
    def get_points(self):
        return self.geometric_map.get_points()
    
    def get_points_and_values(self, threshold=0.5):
        return self.geometric_map.get_points_and_values(threshold=threshold)
    
    def validate_map_boundaries(self, grid_coords):
        return self.geometric_map.validate_map_boundaries(grid_coords)

    def _inflate_obstacles(self, inflation_radius=210):
        self.geometric_map._inflate_obstacles(inflation_radius=inflation_radius)
    
    def get_inflated_map_2d(self, inflation_radius=210):
        return self.geometric_map.get_inflated_map_2d(inflation_radius=inflation_radius)

    def get_frontiers(self):
        return self.geometric_map.get_frontiers()
    
    def expand_map(self, req_grid_coords):
        # Get World Coordinates and Values for each semantic map layer (Room and Object)
        room_layer_coords_and_semantic_value, object_layer_coords_and_semantic_value = self.get_points_and_values_by_semantic_layer()

        # Get New Map Size
        map_size_discretized = self._compute_new_map_size(grid_coords=req_grid_coords)
        map_size_discretized = map_size_discretized.astype(np.int32)
        N, M = map_size_discretized
        
        # Expand Geometric Map
        self.geometric_map.expand_map(req_grid_coords=req_grid_coords)
        print(f"Expanding Semantic Map Descritized Size: {map_size_discretized}")

        self.semantic_layer = np.zeros((N, M, 2))

        new_grid_coords_room_layer = self.batch_world_to_grid_coords(room_layer_coords_and_semantic_value[:, :2])
        new_grid_coords_object_layer = self.batch_world_to_grid_coords(object_layer_coords_and_semantic_value[:, :2])

        self.semantic_layer[new_grid_coords_room_layer[:, 0], new_grid_coords_room_layer[:, 1], self.layer_name_to_idx['room']] = room_layer_coords_and_semantic_value[:, 2]
        self.semantic_layer[new_grid_coords_object_layer[:, 0], new_grid_coords_object_layer[:, 1], self.layer_name_to_idx['object']] = object_layer_coords_and_semantic_value[:, 2]
        
        # Reset Flood Filled Map
        self.flood_filled_map = None
        self.needs_inflation_update = True

    def map_layer_to_coords_and_semantic_values(self, map_layer : np.ndarray):
        idxes = np.where(map_layer > 0)
        xs, ys = idxes
        pc_idxes = np.stack((xs, ys), axis=1)
        pc_coords = self.batch_grid_to_approx_world_coords(pc_idxes)
        pc_semantic_values = map_layer[xs, ys].reshape(-1, 1)
        pc_coords_and_semantic_values = np.hstack((pc_coords, pc_semantic_values))
        return pc_coords_and_semantic_values

    def get_points_and_values_by_semantic_layer(self):
        room_layer_coords_and_semantic_value = self.map_layer_to_coords_and_semantic_values(self.semantic_layer[:, :, self.layer_name_to_idx['room']])
        object_layer_coords_and_semantic_value = self.map_layer_to_coords_and_semantic_values(self.semantic_layer[:, :, self.layer_name_to_idx['object']])
        return room_layer_coords_and_semantic_value, object_layer_coords_and_semantic_value

    ## -- VISUALIZATION FUNCTIONS -- ##
    
    def draw_state(self, ax, state):
        raise NotImplementedError

    def visualize(self, ax):
        self.geometric_map.visualize(ax)

    def visualize_semantic_layer(self, ax, layer):
        ax.imshow(np.rot90(self.semantic_layer[:, :, self.layer_name_to_idx[layer]]))

    def visualize_flood_fill_layer(self, ax, layer):
        assert (self.flood_filled_map is not None), "Before Visualizing Flood Fill Layers, Run flood_fill"
        ax.imshow(np.rot90(self.flood_filled_map[:, :, self.layer_name_to_idx[layer]]))
    
    def visualize_points(self, ax):
        self.geometric_map.visualize_points(ax)

    ## -- GET ROOM AND OBJECT ID FOR SEMANTIC MAPPING FUNCTIONS -- ##

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
        
    def get_room_list(self):
        return [room_name for room_name in self.room_to_id]

    def get_object_list(self):
        return [object_name for object_name in self.object_to_id]

    def get_invalid_room_list(self):
        return list(self.invalid_rooms)

    def get_invalid_object_list(self):
        return list(self.invalid_objects)
        
    def print_item_ids(self):
        print("\nRoom Layer Mappings:")
        for id, room in self.room_to_id.items():
            print(f"{id} -> {room}")
        
        print("\nObject Layer Mappings:")
        for id, object_name in self.object_to_id.items():
            print(f"{id} -> {object_name}")

    ## -- FORMATING IMG SEGMENTATION AND PC DATA FUNCTIONS -- ##

    def format_img_segmentation(self, img_segmentation, labels):
        """
        img_segmentation: (M, N) np.ndarray img with object labels 
        labels: list[tuple] tuple -> (segmenter_object_id, label)
        """
        formatted_segmented_img = np.zeros_like(img_segmentation)

        for segmenter_object_id, object_label in labels:
            if object_label.lower() not in self.invalid_objects:
                semantic_map_object_id = self.get_object_id(object_label)
                formatted_segmented_img[img_segmentation == segmenter_object_id] = semantic_map_object_id
        return formatted_segmented_img

    def label_and_filter_point_cloud(self, pc, formated_segmented_img, room_label):
        """
        Docstring for label_and_format_point_cloud
        :param pc: (N, 3) np.ndarray a 3D Point Cloud
        :param segmented_img: (M, N) np.ndarry with object labels from semantic map

        return: (N, 4) 0: X axis coord, 1: Y axis coord, 2: room id, 3: object id
        """
        formatted_segmented_img_mask = formated_segmented_img != 0
        pc_mask = formatted_segmented_img_mask.flatten()

        object_ids = formated_segmented_img[formatted_segmented_img_mask] # (Q,)
        room_ids = np.ones_like(object_ids) * self.get_room_id(room_label) # (Q,)

        filtered_pc = pc[pc_mask] # (Q, 3)

        # Remove dim 1
        filtered_pc = np.stack((filtered_pc[:, 0], filtered_pc[:, 2]), axis=1) # (Q, 2)

        # TODO: BAD HACK REMOVE THIS SOON
        def align_point_cloud(pc_flattened_coords):
            theta = -np.pi / 2  # 90 degrees in radians
            rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                            [np.sin(theta), np.cos(theta)]])
            pc_flattened_coords = pc_flattened_coords.dot(rotation_matrix.T)
            return pc_flattened_coords

        # TODO HACK: USED TO ROTATE POINT CLOUD TO BE IN CORRECT ORIENTATION FOR WHEN READING (GETS REORIENTED BASED ON STATE LATER)
        filtered_pc = align_point_cloud(filtered_pc) # TODO: TO REMOVE THIS SOON

        pc_and_labels = np.concatenate((filtered_pc, room_ids.reshape(-1, 1), object_ids.reshape(-1, 1)), axis=1)
        return pc_and_labels

    def get_semantic_value_at_grid_coords(self, grid_coords):
        gx, gy = grid_coords
        return self.semantic_layer[gx, gy]
    
    def batch_get_semantic_value_at_grid_coords(self, grid_coords):
        return self.semantic_layer[grid_coords[:, 0], grid_coords[:, 1]]
    
    def get_semantic_flood_fill_value_at_grid_coords(self, grid_coords):
        gx, gy = grid_coords
        return self.flood_filled_map[gx, gy]
    
    def batch_get_semantic_flood_fill_value_at_grid_coords(self, grid_coords):
        return self.flood_filled_map[grid_coords[:, 0], grid_coords[:, 1]]
    
    ## -- UPDATING SEMANTIC AND GEOMETRIC MAP FUNCTIONS -- ##

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
    
    # Not used currently
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

        map_points = self.get_points()
        grid_coords = self.batch_world_to_grid_coords(map_points)
        q = []
        for x, y in grid_coords:
            label = self.semantic_layer[x, y]
            if label[0] > 0:
                q.append((x,y,label))

        self.flood_filled_map = np.copy(self.semantic_layer)
        
        neighbors = [(0,-1),(0,1),(1,0),(-1,0)]
        visited = set()

        min_x, min_y = 0, 0
        max_x, max_y = self.get_map_2d().shape

        if limit_fill_extent:
            min_x, min_y = np.min(grid_coords, axis=0)
            max_x, max_y = np.max(grid_coords, axis=0)

        map_2d = self.get_map_2d()

        while q:
            x, y, label = q.pop(0)
            if (x, y) in visited:
                continue
            visited.add((x,y))

            self.flood_filled_map[x,y] = label
            
            for dx, dy in neighbors:
                nx = x + dx
                ny = y + dy
                
                if nx >= min_x and nx < max_x and ny >= min_y and ny < max_y and map_2d[nx, ny] < 0.5 and (nx, ny) not in visited: #TODO: Check why this improves speed
                    q.append((nx, ny, label))
    
    def _nearest_neighbor_flood_fill(self, limit_fill_extent=False):
        map_points = self.get_points()
        grid_coords = self.batch_world_to_grid_coords(map_points)
        min_grid_coords = np.min(grid_coords, axis=0)
        max_grid_coords = np.max(grid_coords, axis=0)
        kd_tree = KDTree(grid_coords)
        semantic_labels = self.semantic_layer[grid_coords[:, 0], grid_coords[:, 1]]
        grid_xs, grid_ys = np.where(self.get_map_2d() < 0.5)
        grid_coords = np.stack((grid_xs, grid_ys), axis=1)
        if limit_fill_extent:
            semantic_labeling_mask_xs = np.logical_and(grid_coords[:, 0] >= min_grid_coords[0], grid_coords[:, 0] <= max_grid_coords[0])
            semantic_labeling_mask_ys = np.logical_and(grid_coords[:, 1] >= min_grid_coords[1], grid_coords[:, 1] <= max_grid_coords[1])
            semantic_labeling_mask = np.logical_and(semantic_labeling_mask_xs, semantic_labeling_mask_ys)
            grid_coords = grid_coords[semantic_labeling_mask]
        _, idxes = kd_tree.query(grid_coords, k=1)
        self.flood_filled_map = np.copy(self.semantic_layer)
        self.flood_filled_map[grid_coords[:, 0], grid_coords[:, 1]] = semantic_labels[idxes[:, 0]]

    ## -- SEMANTIC SAVE FUNCTIONS -- ##

    def save_raw_semantics(self, map_save_dir, file_name_ext="final"):
        pickle.dump(self.semantic_layer, open(f"{self.get_save_dir(map_save_dir)}/{self.map_type_name}_raw_semantics_{file_name_ext}.pickle", "wb"))
        pickle.dump(self.room_to_id, open(f"{self.get_save_dir(map_save_dir)}/{self.map_type_name}_room_to_id_{file_name_ext}.pickle", "wb"))
        pickle.dump(self.object_to_id, open(f"{self.get_save_dir(map_save_dir)}/{self.map_type_name}_object_to_id_{file_name_ext}.pickle", "wb"))
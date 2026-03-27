import numpy as np

from map import Map
from semantic_map import SemanticMap

class SemanticExplorationMap(SemanticMap):
    def __init__(self, map_obj: Map):
        super().__init__(map_obj=map_obj)
        # self.geometric_map = map_obj
        self.embedding_layer = np.zeros(self.geometric_map.get_shape_2d())
        self.map_type_name = 'semantic_exploration_map'

    def update(self, scan, predicted_state):
        raise NotImplementedError
    
    def update_map(self, aligned_scan, updated_state=None):
        raise NotImplementedError
    
    def extend_map(self, req_grid_coords):
        raise NotImplementedError
    
    def map_layer_to_coords_and_embedding_values(self, map_embedding_layer: np.ndarray):
        idxes = np.where(map_embedding_layer != 0) # TODO: Verify this works
        xs, ys = idxes
        pc_idxes = np.stack((xs, ys), axis=1)
        pc_coords = self.batch_grid_to_approx_world_coords(pc_idxes)
        pc_embedding_values = map_embedding_layer[xs, ys].reshape(-1, 1)
        pc_coords_and_embedding_values = np.hstack((pc_coords, pc_embedding_values))
        return pc_coords_and_embedding_values
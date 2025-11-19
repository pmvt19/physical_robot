import numpy as np

from map import Map

class SemanticMap(Map):
    def __init__(self, map_obj):
        self.map : Map = map_obj
        self.semantic_layer = np.zeros((self.map.shape[0], self.map.shape[1], 2)) # axis 2: 0 -> Room Level Information, 1 -> Object Level Information

    def update_semantic_map(self, semantics):
        """
        semantics: TBD on what input this is

        Maybe a (N, 4) matrix where axis {0,1,2} is the point in 3d world coords and axis {3} is a label??
        Maybe a (N, 5) matrix where axis {0,1,2} is the point in 3d world coords and axis {3, 4} are labels??
        """
        raise NotImplementedError

    def update_map(self, aligned_scan, semantics):
        self.map.update_map(aligned_scan)
    
    def update(self, scan, predicted_state):
        return self.map.update(scan, predicted_state)
    
    def flood_fill(self):
        # Queue with starting seed for classes
        # BFS and labeling classes with closest labels
        # IDEA: Implement this function in C++ and use python bindings here?
        raise NotImplementedError
    
    def visualize(self, ax, color='blue'):
        self.map.visualize(ax, color)
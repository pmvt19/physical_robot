import numpy as np

from map import Map

class SemanticMap(Map):
    def __init__(self, map_obj):
        self.map : Map = map_obj
        self.semantic_map = np.zeros((self.map.shape[:2]))

    def update_map(self, aligned_scan):
        self.map.update_map(aligned_scan)
    
    def update(self, scan, predicted_state):
        return self.map.update(scan, predicted_state)
    
    def visualize(self, ax, color='blue'):
        raise NotImplementedError
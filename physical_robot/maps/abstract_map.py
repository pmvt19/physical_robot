import numpy as np

class AbstractMap():
    def __init__(self):
        pass

    def update_map(self, aligned_scan):
        raise NotImplementedError
    
    def update(self, scan, predicted_state):
        raise NotImplementedError
    
    def visualize(self, ax, color='blue'):
        raise NotImplementedError
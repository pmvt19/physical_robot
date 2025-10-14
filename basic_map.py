import numpy as np
import matplotlib.pyplot as plt
from icp import run_icp

class BasicMap():
    def __init__(self, initial_scan):
        assert(isinstance(initial_scan, np.ndarray))
        # assert(initial_scan.shape[1] == 2)
        
        self.points = initial_scan

    def update_map(self, aligned_scan):
        '''
        Update map points to include aligned_scan points
        '''
        # For now, update map just concatenates the new scan points
        self.points = np.vstack((self.points, aligned_scan))

    def update(self, scan, predicted_state):
        '''
        Compute the Alignment for the scan to the map given the initial state guess: predicted_state

        Returns: updated_state
        '''
        T = run_icp(scan, self.points, predicted_state, visualize=True)
        updated_theta = np.arccos(T[0, 0])

        updated_x = T[0, 2]
        updated_y = T[1, 2]

        aligned_scan = (T@scan.T).T

        self.update_map(aligned_scan)
        return np.array([updated_x, updated_y, updated_theta])

    def visualize(self, ax, color='blue'):
        '''
        Visualizes Current Map
        '''
        # ax.title(self.points.shape)
        ax.set_xlim(np.min(self.points[:, 0])-1000, np.max(self.points[:, 0])+1000)
        ax.set_ylim(np.min(self.points[:, 1])-1000, np.max(self.points[:, 1])+1000)
        ax.scatter(self.points[:, 0], self.points[:, 1], color=color)



    
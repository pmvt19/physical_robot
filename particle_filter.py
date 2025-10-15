import numpy as np
import matplotlib.pyplot as plt
from scipy import ndimage

from utils import pairwise_dists

from test_utils import generate_fake_map

class ParticleFilter():
    def __init__(self, map_obj):
        self.map = map_obj

    def _compute_dist_map(self):
        self.dist_map = np.zeros_like(self.map.map)
        print(self.dist_map.shape)

        xlen, ylen = self.map.map.shape
        print(xlen, ylen)
        x_idxs = np.arange(xlen)
        y_idxs = np.arange(ylen)

        xs, ys = np.meshgrid(x_idxs, y_idxs)
        idxes = np.concatenate((xs.reshape(-1, 1), ys.reshape(-1, 1))) # (num_cells, 2)

        all_cells_values = self.map.map.reshape(-1, 1)

        inverse_map = 1 - self.map.map
        self.dist_map = ndimage.distance_transform_edt(inverse_map)

        fig, (ax1, ax2) = plt.subplots(1, 2)
        ax1.imshow(np.rot90(self.map.map))
        ax2.imshow(np.rot90(self.dist_map))
        plt.show()


        # print(p_dists.shape)

    # TODO: Change function name
    def batch_get_dists(self):
        pass

    def visualize_dist_map(self, ax):
        ax.imshow(np.rot90(self.dist_map))
    
    # 
    def visualize_map(self, ax):
        self.map.visualize(ax)


if __name__ == '__main__':
    mymap = generate_fake_map()
    
    pf = ParticleFilter(mymap)
    pf._compute_dist_map()
    
        
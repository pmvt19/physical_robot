import numpy as np
import matplotlib.pyplot as plt
from scipy import ndimage

from utils import pairwise_dists

from test_utils import generate_fake_map

class ParticleFilter():
    def __init__(self, map_obj):
        self.map = map_obj

    def _compute_dist_map(self):
        inverse_map = 1 - self.map.map
        self.dist_map = ndimage.distance_transform_edt(inverse_map)

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

    fig, (ax1, ax2) = plt.subplots(1, 2)
    pf.visualize_map(ax1)
    pf.visualize_dist_map(ax2)
    plt.show()
    
        
import time

import matplotlib.pyplot as plt
import numpy as np

from physical_robot.maps import Map

points = np.load('test_data/test_map_points.npy')

map = Map()
map.init_map(points)

map.visualize(plt.gca())
plt.show()

inflation_radius = 210
for i in range(10):
    st = time.time()
    inflated_map = map.get_inflated_map_2d(inflation_radius=inflation_radius)
    et = time.time()

    if i>5:
        inflation_radius=300

    print(f"Time to Inflate Obstacles: {et - st}")

def test_grid_coords_to_approx_world_coords():
    pass

def test_world_coords_to_grid_coords():
    pass

def test_expanding_map_does_not_change_world_points():
    pass

def test_inflation_map_is_cached():
    pass

def test_inflation_map_is_recomputed_after_radius_change():
    pass
import numpy as np
from map import Map

def generate_fake_map():
    xs_1 = np.linspace(0, 1000, 10000)
    ys_1 = np.ones_like(xs_1) * 0.0

    ys_2 = np.linspace(0, 2000, 20000)
    xs_2 = np.ones_like(ys_2) * 1000.0

    xs_3 = np.linspace(0, 1000, 10000)
    ys_3 = np.ones_like(xs_3) * 2000.0

    ys_4 = np.linspace(0, 2000, 20000)
    xs_4 = np.ones_like(ys_4) * 0.0

    s1 = np.stack((xs_1, ys_1), axis=1)
    s2 = np.stack((xs_2, ys_2), axis=1)
    s3 = np.stack((xs_3, ys_3), axis=1)
    s4 = np.stack((xs_4, ys_4), axis=1)

    init_points = np.vstack((s1, s2, s3, s4))

    mymap = Map(initial_scan=init_points)
    return mymap
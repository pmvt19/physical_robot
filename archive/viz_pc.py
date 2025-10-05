import numpy as np
import matplotlib.pyplot as plt

pc_paths = ["slam_data/scene_1.npy", "slam_data/scene_2.npy", "slam_data/scene_3.npy", "slam_data/scene_4.npy", "slam_data/scene_5.npy", "slam_data/scene_6.npy"]

for path in pc_paths:
    plt.clf()
    pc = np.load(path)
    plt.scatter(pc[:, 0], pc[:, 1], color='blue')
    plt.show()






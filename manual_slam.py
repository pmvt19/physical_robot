import numpy as np
import matplotlib.pyplot as plt

# pc_paths = ["slam_data/scene_1.npy", "slam_data/scene_2.npy", "slam_data/scene_3.npy", "slam_data/scene_4.npy", "slam_data/scene_5.npy", "slam_data/scene_6.npy"]
pc_paths = ["new_slam_data/scene_1.npy", "new_slam_data/scene_2.npy", "new_slam_data/scene_3.npy", "new_slam_data/scene_4.npy", "new_slam_data/scene_5.npy", "new_slam_data/scene_6.npy"]

def viz_pcs():
    for path in pc_paths:
        plt.clf()
        pc = np.load(path)
        plt.scatter(pc[:, 0], pc[:, 1], color='blue')
        plt.show()
    
# robot_states = np.array([[0.0, 0.0, 0.0],
#                          [807.4, 0.0, 0.0],
#                          [807.4, 0.0, np.pi/2],
#                          [807.4, 451.65, np.pi/2],
#                          [807.4, 451.65, np.pi],
#                          [211.8, 451.65, np.pi]])

# [ 197.0317788  -197.28680311]
# [-1.53354616 -1.54326137]
# [ 444.76238964 -445.88449659]
# [-1.53743224 -1.53888953]
# [ 195.80766214 -197.54182742]

robot_states = np.array([[0.0, 0.0, 0.0],
                         [197.0317788, 0.0, 0.0],
                         [197.0317788, 0.0, 1.53354616],
                         [197.0317788, 444.76238964, 1.53354616],
                         [197.0317788, 444.76238964, 3.0709784],
                         [0.0, 444.76238964, 3.0709784]])


def preprocess_pc(pc):
    return pc #* 10

def compute_tranformation_matrix(robot_state):
    x, y, theta = robot_state
    theta = -theta
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta), np.cos(theta)]])
    t = np.array([[x, -y]])

    T = np.zeros((3, 3))
    T[:2, :2] = R
    T[:2, 2] = t
    T[2, 2] = 1
    return T


def transform_pc(robot_state, pc):
    pc = pc[:, :2]
    x, y, theta = robot_state
    theta = -theta
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta), np.cos(theta)]])
    t = np.array([[x, -y]])
    # print(R.shape, pc.shape, t.shape)
    print(t)
    transformed_pc = (R @ pc.T).T + t
    # print(R.shape, pc.shape, t.shape, transformed_pc.shape)
    return transformed_pc


def viz_transformed_pcs(transformed_pcs, overlay=False):
    if overlay:
        plt.clf()
        for pc in transformed_pcs:
            plt.scatter(pc[:, 0], pc[:, 1])
        plt.show()
    else:
        for pc in transformed_pcs:
            plt.clf()
            plt.scatter(pc[:, 0], pc[:, 1])
            plt.show()

def load_pcs(paths):
    pcs = []
    for path in paths:
        pcs.append(np.load(path))
    return pcs

if __name__ == '__main__':
    # viz_pcs()
    pcs = load_pcs(pc_paths)
    preprocess_pcs = [preprocess_pc(pc) for pc in pcs]
    transformed_pcs = [transform_pc(robot_states[i], pc) for i, pc in enumerate(preprocess_pcs)]
    viz_transformed_pcs(transformed_pcs, overlay=True)

    


import numpy as np
from sklearn.neighbors import KDTree
import matplotlib.pyplot as plt

# source_pc = np.load("new_slam_data/scene_6.npy") # Moved Position
# target_pc = np.load("new_slam_data/scene_5.npy") # Map (Init Position)

source_pc = np.load("slam_run_1/scene_2.npy") # Moved Position
target_pc = np.load("slam_run_1/scene_1.npy") # Map (Init Position)

# slam data
# init_x, init_y, init_theta = 0, 0, 0
# init_x, init_y, init_theta = 0, 0, -np.pi/4
# init_x, init_y, init_theta = 807.4, 0.0, 0.0
# init_x, init_y, init_theta = 807.4, 0.0, np.pi/2
# init_x, init_y, init_theta = 807.4, 0.0, 0.0

# new slam data
# init_x, init_y, init_theta = 197.0317788, 0.0, 0.0
# init_x, init_y, init_theta = 197.0317788, 0.0, 0.0
# init_x, init_y, init_theta = 0.0, 0.0, np.pi/2
# init_x, init_y, init_theta = 0.0, 444.7, 0.0
# init_x, init_y, init_theta = 197.0317788, -444.7, np.pi/2

# init_x, init_y, init_theta = 197.031778, 0.0, 0.0
init_x, init_y, init_theta = 341.58, 140.96, 0.0

init_theta = -init_theta
init_transform = np.array([[np.cos(init_theta), -np.sin(init_theta), init_x],
                           [np.sin(init_theta), np.cos(init_theta),  init_y],
                           [0.0,                0.0,                 1.0]])

cur_T = init_transform
print(cur_T)
print(source_pc.shape, target_pc.shape)

# print(source_pc)
source_pc[:, 2] = 1
# print(source_pc)
# source_pc_homogenous = np.concatenate((source_pc, ))

transformed_source_pc = (cur_T @ source_pc.T).T

print(transformed_source_pc.shape)

plt.xlim(np.min(target_pc[:, 0])-1000, np.max(target_pc[:, 0])+1000)
plt.ylim(np.min(target_pc[:, 1])-1000, np.max(target_pc[:, 1])+1000)
plt.scatter(target_pc[:, 0], target_pc[:, 1], color='red')
plt.scatter(transformed_source_pc[:, 0], transformed_source_pc[:, 1], color='blue')
plt.scatter(source_pc[:, 0], source_pc[:, 1], color='green')

# for si, ti in enumerate(idx):
#     plt.plot([transformed_source_pc[si, 0], target_pc[ti, 0]], [transformed_source_pc[si, 1], target_pc[ti, 1]], color='yellow')

plt.show()
# plt.pause(0.1)
# plt.cla()

def fit_rigid(src, tgt):
    T = np.identity(3)

    src_mean = np.mean(src, axis=0)
    tgt_mean = np.mean(tgt, axis=0)

    src = (src - src_mean)
    tgt = (tgt - tgt_mean)

    C = tgt.T @ src

    U, e, Vt = np.linalg.svd(C, full_matrices=True)

    det = np.linalg.det(U @ Vt)
    # print(U.shape, Vt.shape)
    Rmat = U @ np.diag(np.array([1, det])) @ Vt 

    d = tgt_mean - Rmat @ src_mean


    T[:2, :2] = Rmat
    T[:2, 2] = d
    return T

# cur_T 
src_points = transformed_source_pc


inlier_dist_threshold = 100
num_points = len(transformed_source_pc)
for i in range(15):

    kd_tree = KDTree(target_pc[:, :2])
    dists, idx = kd_tree.query(src_points[:, :2])
    # print(idx)
    dists = dists.flatten()
    # print(dists)
    inlier_masks = dists < inlier_dist_threshold

    num_inliers = np.sum(dists < inlier_dist_threshold)
    inlier_ratio = num_inliers / num_points
    print(dists)
    # src_points = src_points[inlier_masks]

    if inlier_ratio > 0.999:
        print(f"Converged After {i} iterations")
        break

    # print(inlier_ratio, num_inliers, num_points)

    idx = idx.flatten()

    matched_tgt_points = target_pc[idx]

    T_delta = fit_rigid(src_points[:, :2], matched_tgt_points[:, :2])
    # print(T_delta)
    # cur_T = cur_T @ T_delta
    cur_T = T_delta @ cur_T
    

    # print(T)
    # exit()

    # transformed_points = (cur_T @ src_points.T).T
    transformed_points = (T_delta @ src_points.T).T

    src_points = transformed_points

    plt.xlim(np.min(target_pc[:, 0])-1000, np.max(target_pc[:, 0])+1000)
    plt.ylim(np.min(target_pc[:, 1])-1000, np.max(target_pc[:, 1])+1000)
    plt.scatter(target_pc[:, 0], target_pc[:, 1], color='red')
    plt.scatter(transformed_points[:, 0], transformed_points[:, 1], color='blue')
    plt.scatter(source_pc[:, 0], source_pc[:, 1], color='green')

    for si, ti in enumerate(idx):
        plt.plot([transformed_points[si, 0], target_pc[ti, 0]], [transformed_points[si, 1], target_pc[ti, 1]], color='yellow')

    # plt.show()
    plt.title(f"Iteration: {i}, Inlier Ratio: {inlier_ratio}")
    plt.pause(0.1)
    plt.cla()

    # T = fit_rigid(source_pc, target_pc)
src_points = src_points[inlier_masks]
num_points = len(src_points)
for i in range(15):

    kd_tree = KDTree(target_pc[:, :2])
    dists, idx = kd_tree.query(src_points[:, :2])
    # print(idx)
    dists = dists.flatten()
    # print(dists)
    inlier_masks = dists < inlier_dist_threshold

    num_inliers = np.sum(dists < inlier_dist_threshold)
    inlier_ratio = num_inliers / num_points

    # src_points = src_points[inlier_masks]

    # if inlier_ratio > 0.999:
    #     print(f"Converged After {i} iterations")
    #     break

    # print(inlier_ratio, num_inliers, num_points)

    idx = idx.flatten()

    matched_tgt_points = target_pc[idx]

    T_delta = fit_rigid(src_points[:, :2], matched_tgt_points[:, :2])
    # print(T_delta)
    # cur_T = cur_T @ T_delta
    cur_T = T_delta @ cur_T
    

    # print(T)
    # exit()

    # transformed_points = (cur_T @ src_points.T).T
    transformed_points = (T_delta @ src_points.T).T

    src_points = transformed_points

    plt.xlim(np.min(target_pc[:, 0])-1000, np.max(target_pc[:, 0])+1000)
    plt.ylim(np.min(target_pc[:, 1])-1000, np.max(target_pc[:, 1])+1000)
    plt.scatter(target_pc[:, 0], target_pc[:, 1], color='red')
    plt.scatter(transformed_points[:, 0], transformed_points[:, 1], color='blue')
    plt.scatter(source_pc[:, 0], source_pc[:, 1], color='green')

    for si, ti in enumerate(idx):
        plt.plot([transformed_points[si, 0], target_pc[ti, 0]], [transformed_points[si, 1], target_pc[ti, 1]], color='yellow')

    # plt.show()
    plt.title(f"Iteration: {i} v2, Inlier Ratio: {inlier_ratio}")
    plt.pause(0.1)
    plt.cla()

print(cur_T)

print(f"Theta: {np.arccos(cur_T[0, 0])}")

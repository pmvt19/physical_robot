import numpy as np
from sklearn.neighbors import KDTree
import matplotlib.pyplot as plt

def get_init_transformation_matrix(state):
    init_x, init_y, init_theta = state

    init_theta = init_theta
    init_transform = np.array([[np.cos(init_theta), -np.sin(init_theta), init_x],
                            [np.sin(init_theta), np.cos(init_theta),  init_y],
                            [0.0,                0.0,                 1.0]])
    return init_transform

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

def run_icp(source_pc, target_pc, predicted_state, filter_init_outliers=True, visualize=False):

    cur_T = get_init_transformation_matrix(predicted_state)
    print("Original T")
    print(cur_T)
    source_pc[:, 2] = 1

    transformed_source_pc = (cur_T @ source_pc.T).T

    if visualize:
        plt.title("<PLACEHOLDER>")
        plt.xlim(np.min(target_pc[:, 0])-1000, np.max(target_pc[:, 0])+1000)
        plt.ylim(np.min(target_pc[:, 1])-1000, np.max(target_pc[:, 1])+1000)
        plt.scatter(target_pc[:, 0], target_pc[:, 1], color='red', label='map')
        plt.scatter(transformed_source_pc[:, 0], transformed_source_pc[:, 1], color='blue', label='scan')
        plt.legend()
        # plt.scatter(source_pc[:, 0], source_pc[:, 1], color='green')
        plt.show()

    src_points = transformed_source_pc

    if filter_init_outliers:
        kd_tree = KDTree(target_pc[:, :2])
        dists, idx = kd_tree.query(src_points[:, :2])
        dists = dists.flatten()
        init_inlier_dist_threshold = 200
        init_inlier_masks = dists < init_inlier_dist_threshold
        # print(f"Inliers Remaining: {np.sum(init_inlier_masks)}") # TODO: Health Indicator could go here?
        src_points = src_points[init_inlier_masks]


    inlier_dist_threshold = 100
    num_points = len(transformed_source_pc)
    for i in range(15):

        kd_tree = KDTree(target_pc[:, :2])
        dists, idx = kd_tree.query(src_points[:, :2])

        dists = dists.flatten()
        inlier_masks = dists < inlier_dist_threshold

        num_inliers = np.sum(dists < inlier_dist_threshold)
        inlier_ratio = num_inliers / num_points

        if inlier_ratio > 0.999:
            print(f"Converged After {i} iterations")
            break

        idx = idx.flatten()

        matched_tgt_points = target_pc[idx]

        T_delta = fit_rigid(src_points[:, :2], matched_tgt_points[:, :2])

        cur_T = T_delta @ cur_T

        transformed_points = (T_delta @ src_points.T).T

        src_points = transformed_points

        if visualize:
            plt.xlim(np.min(target_pc[:, 0])-1000, np.max(target_pc[:, 0])+1000)
            plt.ylim(np.min(target_pc[:, 1])-1000, np.max(target_pc[:, 1])+1000)
            plt.scatter(target_pc[:, 0], target_pc[:, 1], color='red', label='map')
            plt.scatter(transformed_points[:, 0], transformed_points[:, 1], color='blue', label='t_scan')
            # plt.scatter(source_pc[:, 0], source_pc[:, 1], color='green')

            for si, ti in enumerate(idx):
                plt.plot([transformed_points[si, 0], target_pc[ti, 0]], [transformed_points[si, 1], target_pc[ti, 1]], color='yellow')
            plt.legend()
            plt.title(f"Iteration: {i}, Inlier Ratio: {inlier_ratio}")
            plt.pause(0.1)
            plt.cla()

    src_points = src_points[inlier_masks]
    num_points = len(src_points)
    for i in range(15):

        kd_tree = KDTree(target_pc[:, :2])
        dists, idx = kd_tree.query(src_points[:, :2])

        dists = dists.flatten()
        inlier_masks = dists < inlier_dist_threshold

        num_inliers = np.sum(dists < inlier_dist_threshold)
        inlier_ratio = num_inliers / num_points

        idx = idx.flatten()

        matched_tgt_points = target_pc[idx]

        T_delta = fit_rigid(src_points[:, :2], matched_tgt_points[:, :2])
        cur_T = T_delta @ cur_T

        transformed_points = (T_delta @ src_points.T).T

        src_points = transformed_points

        if visualize:
            plt.xlim(np.min(target_pc[:, 0])-1000, np.max(target_pc[:, 0])+1000)
            plt.ylim(np.min(target_pc[:, 1])-1000, np.max(target_pc[:, 1])+1000)
            plt.scatter(target_pc[:, 0], target_pc[:, 1], color='red', label='map')
            plt.scatter(transformed_points[:, 0], transformed_points[:, 1], color='blue', label='scan')
            # plt.scatter(source_pc[:, 0], source_pc[:, 1], color='green')

            for si, ti in enumerate(idx):
                plt.plot([transformed_points[si, 0], target_pc[ti, 0]], [transformed_points[si, 1], target_pc[ti, 1]], color='yellow')
            plt.legend()
            plt.title(f"Iteration: {i} v2, Inlier Ratio: {inlier_ratio}")
            plt.pause(0.1)
            plt.cla()
    print("Final T")
    print(cur_T)
    return cur_T

def run_single_icp(source_pc, target_pc, init_T, num_iter=15, inlier_dist_threshold=100, visualize=False):

    src_points = (init_T @ source_pc.T).T

    num_points = len(source_pc)
    for i in range(num_iter):
        kd_tree = KDTree(target_pc[:, :2])
        dists, idx = kd_tree.query(src_points[:, :2])

        dists = dists.flatten()
        inlier_masks = dists < inlier_dist_threshold

        num_inliers = np.sum(dists < inlier_dist_threshold)
        inlier_ratio = num_inliers / num_points

        if inlier_ratio > 0.999:
            print(f"Converged After {i} iterations")
            break

        idx = idx.flatten()

        matched_tgt_points = target_pc[idx]

        T_delta = fit_rigid(src_points[:, :2], matched_tgt_points[:, :2])

        cur_T = T_delta @ cur_T

        transformed_points = (T_delta @ src_points.T).T

        src_points = transformed_points

        if visualize:
            plt.xlim(np.min(target_pc[:, 0])-1000, np.max(target_pc[:, 0])+1000)
            plt.ylim(np.min(target_pc[:, 1])-1000, np.max(target_pc[:, 1])+1000)
            plt.scatter(target_pc[:, 0], target_pc[:, 1], color='red')
            plt.scatter(transformed_points[:, 0], transformed_points[:, 1], color='blue')
            plt.scatter(source_pc[:, 0], source_pc[:, 1], color='green')

            for si, ti in enumerate(idx):
                plt.plot([transformed_points[si, 0], target_pc[ti, 0]], [transformed_points[si, 1], target_pc[ti, 1]], color='yellow')

            plt.title(f"Iteration: {i}, Inlier Ratio: {inlier_ratio}")
            plt.pause(0.1)
            plt.cla()
    

if __name__ == '__main__':
    # state = np.array([399.90361422, 0, 0])
    state = np.array([299.90361422, 0, np.deg2rad(2)]) # State is stale for the current loaded scans
    tgt = np.load('data/parking_scene/scan_0.npy')
    src = np.load('data/parking_scene/scan_1.npy')
    run_icp(src, tgt, state, filter_init_outliers=True, visualize=True)
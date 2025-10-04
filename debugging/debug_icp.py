import numpy as np
from sklearn.neighbors import KDTree
import matplotlib.pyplot as plt

def plot_pcs(ax, src, tgt):
    ax.scatter(src[:, 0], src[:, 1], color='blue')
    ax.scatter(tgt[:, 0], tgt[:, 1], color='red')

source_pc = np.load("source_points.npy").astype(np.float64) # Moved Position
target_pc = np.load("target_points.npy").astype(np.float64) # Map (Init Position)
source_loc = np.array([0.0, 0.0, 1.0])
plot_pcs(plt.gca(), source_pc, target_pc)
plt.show()

# init_x, init_y, init_theta = 0, 0, 0
init_x, init_y, init_theta = 0, 0, -np.pi/4
init_transform = np.array([[np.cos(init_theta), -np.sin(init_theta), 0.0],
                           [np.sin(init_theta), np.cos(init_theta),  0.0],
                           [0.0,                0.0,                 1.0]])

cur_T = init_transform
print(source_pc.shape, target_pc.shape)

# print(source_pc)
source_pc[:, 2] = 1
# print(source_pc)
# source_pc_homogenous = np.concatenate((source_pc, ))

transformed_source_pc = (cur_T @ source_pc.T).T

print(transformed_source_pc.shape)

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
    # d = tgt_mean - src_mean

    # print("D", d, tgt_mean, Rmat, src_mean)
    T[:2, :2] = Rmat
    T[:2, 2] = d
    return T

# cur_T 
src_points = transformed_source_pc
src_loc = cur_T @ source_loc

inlier_dist_threshold = 15
num_points = len(transformed_source_pc)

Ts = [init_transform]
dTs = [init_transform]

for i in range(50):

    kd_tree = KDTree(target_pc[:, :2])
    dists, idx = kd_tree.query(src_points[:, :2])
    # print(idx)
    dists = dists.flatten()
    # print(dists)
    num_inliers = np.sum(dists < inlier_dist_threshold)
    inlier_ratio = num_inliers / num_points

    if inlier_ratio > 0.75:
        print(f"Converged After {i} iterations")
        break

    # print(inlier_ratio, num_inliers, num_points)

    idx = idx.flatten()

    matched_tgt_points = target_pc[idx]

    T_delta = fit_rigid(src_points[:, :2], matched_tgt_points[:, :2])

    # cur_T = cur_T @ T_delta
    cur_T = T_delta @ cur_T
    Ts.append(cur_T)
    dTs.append(T_delta)

    # print(cur_T)
    # print(T_delta)

    # print(T)
    # exit()

    # transformed_points = (cur_T @ src_points.T).T
    transformed_points = (T_delta @ src_points.T).T

    src_points = transformed_points
    src_loc = T_delta @ src_loc
    print(cur_T[:2, 2], src_loc)

    print(T_delta)

    plt.xlim(np.min(target_pc[:, 0])-1000, np.max(target_pc[:, 0])+1000)
    plt.ylim(np.min(target_pc[:, 1])-1000, np.max(target_pc[:, 1])+1000)
    plt.scatter(target_pc[:, 0], target_pc[:, 1], color='red')
    plt.scatter(transformed_points[:, 0], transformed_points[:, 1], color='blue')
    plt.scatter(source_pc[:, 0], source_pc[:, 1], color='green')

    plt.scatter(0, 0, color='orange', marker='*')
    plt.scatter(src_loc[0], src_loc[1], color='pink', marker='*')
    

    for si, ti in enumerate(idx):
        plt.plot([transformed_points[si, 0], target_pc[ti, 0]], [transformed_points[si, 1], target_pc[ti, 1]], color='yellow')

    # plt.show()
    plt.pause(0.1)
    plt.cla()

    # T = fit_rigid(source_pc, target_pc)

print(cur_T)

print(f"Theta (cos): {np.arccos(cur_T[0, 0])}")
print(f"Theta (sin): {np.arcsin(cur_T[1, 0])}")
print(f"Translation: {src_loc}")

plt.clf()
plt.cla()
plt.xlim(np.min(target_pc[:, 0])-1000, np.max(target_pc[:, 0])+1000)
plt.ylim(np.min(target_pc[:, 1])-1000, np.max(target_pc[:, 1])+1000)
plt.scatter(target_pc[:, 0], target_pc[:, 1], color='red')
plt.scatter(transformed_points[:, 0], transformed_points[:, 1], color='blue')
plt.scatter(source_pc[:, 0], source_pc[:, 1], color='green')

full_transformed_points = (cur_T @ source_pc.T).T
plt.scatter(full_transformed_points[:, 0], full_transformed_points[:, 1], color='purple')

plt.scatter(0, 0, color='orange', marker='*')
plt.scatter(src_loc[0], src_loc[1], color='pink', marker='*')
plt.scatter(cur_T[0, 2], cur_T[1, 2], color='lime', marker='*')
plt.show()


def compute_final_transform(dTs, Ts):
    t = dTs[0]
    for i in range(1, len(dTs)):
        t = t @ dTs[i]
    # print(t)

    carry_point = np.array([0.0, 0.0, 1.0])
    abs_point = np.array([0.0, 0.0, 1.0])
    for i in range(len(dTs)):
        mt = Ts[i]
        mdt = dTs[i]

        carry_point = mdt @ carry_point
        print(mt@abs_point, carry_point)

compute_final_transform(dTs, Ts)
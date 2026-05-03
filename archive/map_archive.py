import numpy as np

def _compute_grid_centers(self):
    x_idxes = np.arange(self.map.shape[0])
    y_idxes = np.arange(self.map.shape[1])

    xs, ys = np.meshgrid(x_idxes, y_idxes)
    all_idxes = np.stack((xs.flatten(), ys.flatten()), axis=1)
    self.grid_centers = self.batch_grid_to_approx_world_coords(all_idxes) + (self.resolution / 2)

def set_known_clear(self, aligned_scan, updated_state):
    x, y = updated_state
    s_state = np.array([x, y]).reshape(-1, 1)
    repeated_state = np.repeat(s_state, len(aligned_scan), axis=1)
    line_segments = np.hstack((repeated_state, aligned_scan))
    dist_mask = self.compute_close_cells(line_segments)
    self.map[dist_mask] = 0.0
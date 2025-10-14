import math
import time
import numpy as np
import matplotlib.pyplot as plt

from map import Map
from test_utils import generate_fake_map

class SimulatedLidar():
    def __init__(self, map_obj, angular_resolution, max_dist):
        self.map : Map = map_obj

        self.angular_resolution = angular_resolution
        self.max_dist = max_dist

    ### SHOULD BE GENERIC UTILS FUNCTIONS ###
    def batch_line_segments_to_batch_points_dist(self, line_segment_eps, points):
        """
        line_segment_eps: (N, 4)
        points          : (M, 2)

        return: dists (N, M) distance matrix
        """
        pass

    def point_segment_distance(self, segments: np.ndarray, points: np.ndarray) -> np.ndarray:
        """
        Calculates the shortest distance from each point to each line segment.

        This function is fully vectorized using NumPy broadcasting to ensure high performance
        with large numbers of points and segments.

        Args:
            segments: A NumPy array of shape (N, 4) where each row represents a line
                    segment with coordinates (x1, y1, x2, y2).
            points: A NumPy array of shape (M, 2) where each row represents a point
                    with coordinates (x, y).

        Returns:
            A NumPy array of shape (N, M) where the element at index (i, j) is the
            shortest distance from the i-th line segment to the j-th point.
        """
        # Validate input shapes
        if segments.ndim != 2 or segments.shape[1] != 4:
            raise ValueError(f"Segments array must have shape (N, 4), but got {segments.shape}")
        if points.ndim != 2 or points.shape[1] != 2:
            raise ValueError(f"Points array must have shape (M, 2), but got {points.shape}")

        # --- Vectorized Calculation ---

        # Reshape arrays to leverage broadcasting.
        # Segments p1 becomes (N, 1, 2)
        p1 = segments[:, np.newaxis, :2]
        # Segments p2 becomes (N, 1, 2)
        p2 = segments[:, np.newaxis, 2:]
        # Points becomes (1, M, 2)
        pts = points[np.newaxis, :, :]

        # Calculate vectors for segments and from segment start to each point.
        # line_vec is the vector from p1 to p2 for each segment. Shape: (N, 1, 2)
        line_vec = p2 - p1
        # point_vec is the vector from p1 to each point. Shape: (N, M, 2)
        point_vec = pts - p1

        # Calculate the squared length of each line segment.
        # This is equivalent to dot(line_vec, line_vec). Shape: (N, 1)
        line_len_sq = np.sum(line_vec**2, axis=2)

        # Handle the case of zero-length segments (p1 = p2).
        # To avoid division by zero, we replace 0s with 1s. The dot product below
        # will be zero in this case, correctly resulting in t=0.
        line_len_sq[line_len_sq == 0] = 1.0

        # Project point_vec onto line_vec to find the parameter 't'.
        # t represents how far along the line the projection falls.
        # t = dot(point_vec, line_vec) / dot(line_vec, line_vec)
        t = np.sum(point_vec * line_vec, axis=2) / line_len_sq

        # Clamp 't' to the range [0, 1].
        # If t < 0, the closest point is p1.
        # If t > 1, the closest point is p2.
        # If 0 <= t <= 1, the closest point is the projection on the segment.
        t_clamped = np.clip(t, 0, 1)

        # Calculate the coordinates of the closest point on each segment.
        # This uses the clamped 't' to ensure the point is on the segment.
        # Reshape t_clamped to (N, M, 1) for broadcasting with line_vec (N, 1, 2).
        closest_points = p1 + t_clamped[..., np.newaxis] * line_vec

        # Calculate the Euclidean distance from each original point to its
        # corresponding closest point on the segment.
        distances = np.linalg.norm(pts - closest_points, axis=2)

        return distances

    def point_to_points_distance(self, point, points):
        return np.linalg.norm(points - point, axis=1).reshape(1, -1)
    
    def batch_points_to_batch_points_distance(self, batch_points1, batch_points2):
        dists = np.sqrt(np.sum(batch_points1**2, axis=1, keepdims=True) + np.sum(batch_points2**2, axis=1, keepdims=True).T + (-2 * (batch_points1 @ batch_points2.T)))
        return dists
    ### SHOULD BE GENERIC UTILS FUNCTIONS ###
        
    def simulate_lidar(self, loc : np.ndarray):
        x, y, theta = loc # (Thinking) In practice, only x and y matter and we can rotate to handle theta
        state = np.array([x, y])
        angles = np.linspace(0, 2*np.pi, self.angular_resolution)
        coses = np.cos(angles)
        sines = np.sin(angles)

        max_dist_points = np.stack((coses, sines), axis=1) * self.max_dist
        translated_max_dist_points = max_dist_points + state

        reshaped_loc = loc.reshape(1, -1)
        repeated_locs = np.repeat(reshaped_loc, self.angular_resolution, axis=0)
        line_segment_eps = np.concatenate((repeated_locs[:, :2], translated_max_dist_points), axis=1)

        map_points = self.map.get_points()
        dists = self.point_segment_distance(line_segment_eps, map_points)
        dist_mask = dists > (self.map.resolution/2 * math.sqrt(2))
        dists[dist_mask] = np.inf

        # dists is a matrix of viable candidates, we need to choose the one with the smallest dist to loc
        # NOT the one with the smallest dist to the line segment (which is what's current happening)
        # point_to_points_dist() -> (1, M)

        points_dist = self.point_to_points_distance(state, map_points)
        repeated_points_dist = np.repeat(points_dist, self.angular_resolution, axis=0)

        rpd_mask = dists > (self.map.resolution/2 * math.sqrt(2))
        print("HERE")
        print(repeated_points_dist.shape, rpd_mask.shape)
        repeated_points_dist[rpd_mask] = np.inf

        candidate_readings = np.argmin(repeated_points_dist, axis=1)
        candidate_readings_mask = np.min(repeated_points_dist, axis=1) < self.max_dist
        final_readings = candidate_readings[candidate_readings_mask]

        # Get only unique points
        point_idxes = np.unique(final_readings)
        ## IMPORTANT ## Points need to rotate theta units
        unrotated_points = map_points[point_idxes]

        centered_unrotated_points = unrotated_points - state
        R = np.array([[np.cos(-theta), -np.sin(-theta)],
                      [np.sin(-theta), np.cos(-theta)]])
        # print(R.shape, centered_unrotated_points.shape)
        rotated_points = (R @ centered_unrotated_points.T).T
        translated_rotated_points = rotated_points + state
        # return map_points[point_idxes], line_segment_eps, map_points
        return translated_rotated_points, line_segment_eps, map_points
    
    ## -- Batch Functions -- ##

    def batch_simulate_lidar(self, locs : np.ndarray):
        """
        locs : (N, 3) # Batch Location and Orientation of Lidar Sensors
        """
        N, _ = locs.shape
        angles = np.linspace(0, 2*np.pi, self.angular_resolution) # (self.angular_resolution,)
        coses = np.cos(angles) # (self.angular_resolution,)
        sines = np.sin(angles) # (self.angular_resolution,)
        max_dist_points = np.stack((coses, sines), axis=1) * self.max_dist # (self.angular_resolution,2)
        max_dist_points = max_dist_points.reshape(1, self.angular_resolution, 2)

        

        locs_pos = locs[:, :2] # (N, 2)
        locs_pos = locs_pos.reshape(N, 1, 2) # (N, 1, 2)

        batch_translated_max_dist_points = max_dist_points + locs_pos # (N, self.angular_resolution, 2)
        repeated_locs_pos = np.repeat(locs_pos, self.angular_resolution, axis=1) # (N, self.angular_resolution, 2)
        batch_line_segment_eps = np.concatenate((repeated_locs_pos, batch_translated_max_dist_points), axis=2) # (N, self.angular_resolution, 4)


        map_points = self.map.get_points()
        num_map_points = len(map_points)

        reshaped_batch_line_segments_eps = batch_line_segment_eps.reshape(-1, 4)
        dists = self.point_segment_distance(reshaped_batch_line_segments_eps, map_points)
        dists = dists.reshape(N, self.angular_resolution, num_map_points) # (N, self.angular_resolution, num_map_points)
        dists_mask = dists > (self.map.resolution/2 * math.sqrt(2))
        dists[dists_mask] = np.inf

        points_dists = self.batch_points_to_batch_points_distance(locs[:, :2], map_points).reshape(N, 1, num_map_points)
        repeated_points_dists = np.repeat(points_dists, self.angular_resolution, axis=1)
        print(points_dists.shape, dists.shape)
        rpd_mask = dists > (self.map.resolution/2 * math.sqrt(2))
        repeated_points_dists[rpd_mask] = np.inf

        # Check the math on this part
        # candidate_readings actually contains all the points for all the angles that are intersected with the lidar rays
        # The issue is that we want to split and filter them based on max_dists since we don't want to keep points beyond the max dist
        candidate_readings = np.argmin(repeated_points_dists, axis=2) # (N, self.angular_resolution)
        candidate_readings_mask = np.min(repeated_points_dists, axis=2) < self.max_dist # (N, self.angular_resolution)

        # batch_final_readings = []
        # for i in range(N):
        #     batch_final_readings
        batch_final_readings = [map_points[np.unique(candidate_readings[i, candidate_readings_mask[i]])] for i in range(N)]

        # print(candidate_readings_mask.shape, candidate_readings.shape)
        # final_readings = candidate_readings[candidate_readings_mask] 

        # TODO BUG IMPORTANT: Points are currently unrotated

        # print(final_readings.shape)
        print([bfr.shape for bfr in batch_final_readings])

        for i in range(N):
            plt.scatter(map_points[:, 0], map_points[:, 1], color='green')
            plt.scatter(batch_final_readings[i][:, 0], batch_final_readings[i][:, 1], color='blue', zorder=2)
            plt.scatter(locs[i, 0], locs[i, 1], color='red')
            
            plt.gca().set_aspect("equal")
            plt.show()

        return batch_final_readings








if __name__ == '__main__':
    # Create Fake Map for testing
    mymap = generate_fake_map()
    mymap.visualize(plt.gca())
    plt.show()

    sl = SimulatedLidar(map_obj=mymap, angular_resolution=100, max_dist=10000) # Set this to RPLIDAR Range

    state = np.array([-1000.0, 2500.0, np.pi/2])
    points, segments, map_points = sl.simulate_lidar(state)
    grid_coords = mymap.world_to_grid_coords(state[:2])
    print("State to grid coords", (grid_coords))

    plt.scatter(points[:, 0], points[:, 1])
    plt.scatter(state[0], state[1], color='red')
    plt.gca().set_aspect("equal")
    plt.show()



    for x1, y1, x2, y2 in segments:
        plt.plot([x1,x2], [y1, y2], color='yellow')
    plt.scatter(map_points[:, 0], map_points[:, 1], color='green')
    plt.scatter(state[0], state[1], color='red')
    plt.scatter(points[:, 0], points[:, 1], zorder=2)
    plt.gca().set_aspect("equal")
    plt.show()

    points_batch = np.random.uniform(low=np.array([-1500.0, -1500.0, 0.0]), high=np.array([4000.0, 4000.0, 2*np.pi]), size=(10,3))
    sl.batch_simulate_lidar(points_batch)

    exit()

    from sklearn.neighbors import KDTree
    def compute_mse(scanned_points, simulated_points):
        kd_tree = KDTree(scanned_points[:, :2])
        dists, _ = kd_tree.query(simulated_points[:, :2])
        return np.mean(dists)

    particles = np.random.uniform(low=np.array([-1500.0, -1500.0, 0.0]), high=np.array([4000.0, 4000.0, 2*np.pi]), size=(5000,3))
    print(particles.shape)

    dists = np.linalg.norm(particles[:, :2] - state[:2], axis=1)
    simulated_best_state = particles[np.argmin(dists)]
    print(f"Actual State: {state}")
    print(f"Theoretical Best State: {simulated_best_state}")


    read_points, _, _ = sl.simulate_lidar(state)

    st = time.time() 
    lidar_outputs = []
    mses = []
    for i in range(len(particles)):
        # print(f"Idx: {i}", end='\r')
        cur_state = particles[i]
        points, _, _ = sl.simulate_lidar(cur_state)
        lidar_outputs.append(points)
        mse = compute_mse(read_points, points)
        mses.append(mse)

    mses = np.array(mses)

    idx = np.argmin(mses)
    print("optimal reading state:", particles[idx])

        

        
    et = time.time()
    print(f"Time to simulate lidar for {len(particles)} points: {et-st}")




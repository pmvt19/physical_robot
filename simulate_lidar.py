import math
import numpy as np
import matplotlib.pyplot as plt

from map import Map
from test_utils import generate_fake_map


# Create Fake Map for testing
mymap = generate_fake_map()
mymap.visualize(plt.gca())
plt.show()

class SimulatedLidar():
    def __init__(self, map_info, angular_resolution, max_dist):
        self.map : Map = map_info

        self.angular_resolution = angular_resolution
        self.max_dist = max_dist

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
    
    def simulate_lidar(self, loc : np.ndarray):
        x, y, theta = loc # (Thinking) In practice, only x and y matter and we can rotate to handle theta
        state = np.array([x, y])
        angles = np.linspace(0, 2*np.pi, self.angular_resolution)
        coses = np.cos(angles)
        sines = np.sin(angles)

        max_dist_points = np.stack((coses, sines), axis=1) * self.max_dist

        reshaped_loc = loc.reshape(1, -1)
        repeated_locs = np.repeat(reshaped_loc, self.angular_resolution, axis=0)
        line_segment_eps = np.concatenate((repeated_locs[:, :2], max_dist_points), axis=1)

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

    def batch_simulate_lidar(self, locs : np.ndarray):
        pass

sl = SimulatedLidar(map_info=mymap, angular_resolution=360, max_dist=3000) # Set this to RPLIDAR Range
# state = np.array([200.0, -20.0, 0.0])
# state = np.array([-3000.0, 3000.0, 0.0])
state = np.array([-1000.0, 2500.0, np.pi/2])
points, segments, map_points = sl.simulate_lidar(state)
grid_coords = mymap.world_to_grid_coords(state[:2])
print("State to grid coords", (grid_coords))

plt.scatter(points[:, 0], points[:, 1])
plt.scatter(state[0], state[1], color='red')
plt.gca().set_aspect("equal")
plt.show()



# for x1, y1, x2, y2 in segments:
#     plt.plot([x1,x2], [y1, y2], color='yellow')
plt.scatter(map_points[:, 0], map_points[:, 1], color='green')
plt.scatter(state[0], state[1], color='red')
plt.scatter(points[:, 0], points[:, 1], zorder=2)
plt.gca().set_aspect("equal")
plt.show()




        

        

        



    

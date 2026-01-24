import logging
import time
import numpy as np
from shapely import Polygon

def register_logger(logger_name, log_filename, level=logging.INFO, std_err=False):
    logger = logging.getLogger(logger_name)
    logger.setLevel(level)
    handler = logging.FileHandler(f"logs/{log_filename}.log", mode="w")
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    if std_err:
        logger.addHandler(logging.StreamHandler())
    return logger

def pairwise_dists(points1, points2):
    dists = np.sqrt(np.sum(points1**2, axis=1, keepdims=True) + np.sum(points2**2, axis=1, keepdims=True).T + (-2 * (points1 @ points2.T)))
    return dists

def create_rectangle_geometry(x_loc, y_loc, x_width, y_length):
    shape = Polygon([[x_loc-x_width/2, y_loc-y_length/2], 
                        [x_loc-x_width/2, y_loc+y_length/2],
                        [x_loc+x_width/2, y_loc+y_length/2],
                        [x_loc+x_width/2, y_loc-y_length/2],])
    return shape

def transformation_mat_to_state(T):
    theta = np.arctan2(T[1,0],T[0,0]) % (2*np.pi)
    x = T[0, 2]
    y = T[1, 2]
    return np.array([x, y, theta])

def timer(base_fn):
    def enhanced_fn(*args, **kwargs):
          st = time.time()
          result = base_fn(*args, **kwargs)
          et = time.time()
          print(f"Time to run {base_fn.__name__}: {et - st} secs")
          return result
    return enhanced_fn

### Geometry Utils ###
def batch_line_segments_to_batch_points_dist(self, line_segment_eps, points):
        """
        line_segment_eps: (N, 4)
        points          : (M, 2)

        return: dists (N, M) distance matrix
        """
        pass

def point_segment_distance(segments: np.ndarray, points: np.ndarray) -> np.ndarray:
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

def line_seg_to_points_dist(p1: np.ndarray, p2: np.ndarray, points: np.ndarray) -> np.ndarray:
    """
    Compute the shortest distance between a line segment (p1, p2) and a set of points.

    Parameters
    ----------
    p1 : np.ndarray
        Starting point of the line segment, shape (d,)
    p2 : np.ndarray
        Ending point of the line segment, shape (d,)
    points : np.ndarray
        Array of points, shape (N, d)

    Returns
    -------
    np.ndarray
        Distances from each point to the line segment, shape (N,)
    """
    # Vector along the line segment
    seg_vec = p2 - p1
    seg_len_sq = np.dot(seg_vec, seg_vec)

    # Vectors from p1 to the points
    p1_to_points = points - p1

    # Project each point onto the line, normalized by segment length
    t = np.einsum('ij,j->i', p1_to_points, seg_vec) / seg_len_sq

    # Clamp t to [0,1] to stay within the segment
    t = np.clip(t, 0.0, 1.0)

    # Closest point on the segment for each point
    proj_points = p1 + np.outer(t, seg_vec)

    # Distances to the closest points
    dists = np.linalg.norm(points - proj_points, axis=1)

    return dists

def point_to_points_distance(point, points):
    return np.linalg.norm(points - point, axis=1).reshape(1, -1)

def batch_points_to_batch_points_distance(batch_points1, batch_points2):
    dists = np.sqrt(np.sum(batch_points1**2, axis=1, keepdims=True) + np.sum(batch_points2**2, axis=1, keepdims=True).T + (-2 * (batch_points1 @ batch_points2.T)))
    return dists

### Image Utils ###
def create_circular_kernel(n, radius=None):
    # Default radius to n/2 if not specified
    if radius is None:
        radius = n / 2
    
    # 1. Create a coordinate grid (0 to n-1)
    y, x = np.ogrid[:n, :n]
    
    # 2. Find the center coordinates
    center = (n - 1) / 2
    
    # 3. Calculate squared distance from center
    dist_from_center = (x - center)**2 + (y - center)**2
    
    # 4. Create binary mask (True where distance <= radius^2)
    kernel = dist_from_center <= radius**2
    
    return kernel.astype(np.uint8)

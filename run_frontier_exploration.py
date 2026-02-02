from robot import Robot
from basic_map import BasicMap
from map import Map
from advanced_map import AdvancedMap
from semantic_map import SemanticMap
import numpy as np
import matplotlib.pyplot as plt
import pickle
import time
import os
from vlm_client import VLMClient
from image_segmentation import ImageSegmenter
from prompts import ASSIGN_ROOM_LABEL_ONLY_PROMPT, ASSIGN_ROOM_LABEL
from vlm_output_schema import RoomLabel
from sklearn.neighbors import KDTree
from heapq import *
from utils import create_local_mask
        
def dijkstra(start, targets, grid):
    N, M = grid.shape

    sx, sy = start
    q = []
    heappush(q, ((0.0, None, (sx, sy))))

    visited = set()

    neighbors = [(0,1), (1,0), (0,-1), (-1,0)]

    child_to_parent = {(sx, sy) : None}

    frontier_target = None

    while q:
        cost, parent, (x, y) = heappop(q)

        if (x, y) in visited:
            continue
        visited.add((x,y))
        child_to_parent[(x,y)] = parent

        if (x, y) in targets:
            print("found_target")
            frontier_target = (x, y)
            break

        for ox, oy in neighbors:
            nx = x + ox
            ny = y + oy

            if nx >= 0 and nx < N and ny >= 0 and ny < M and grid[nx, ny] <= 0.5:
                heappush(q, ((cost + grid[nx, ny], (x, y), (nx, ny))))
    print(f"Length of Visited: {len(visited)}")

    if frontier_target is not None and frontier_target in child_to_parent:
        current = frontier_target
        path = []
        while current:
            path.append(current)
            current = child_to_parent[current]
        
        return path[::-1]
    else:
        return None


def get_path_to_frontier_robust(map: AdvancedMap, robot_state: np.ndarray):
    assert robot_state.shape[0] == 2
    print(f"Getting path to frontier with these args: {robot_state}")

    map_2d = map.get_inflated_map_2d()

    grid_robot_state = map.world_to_grid_coords(robot_state[:2])

    # Clear start radius
    metric_radius = 100
    grid_radius = metric_radius / map.resolution
    local_circle_mask = create_local_mask(map_2d.shape, grid_robot_state, grid_radius)
    map_2d[local_circle_mask] = 0 # TODO: Make a copy??


    frontiers = map.get_frontiers()
    grid_frontier_targets = map.batch_world_to_grid_coords(frontiers)
    set_of_grid_frontier_targets = set([(i,j) for i,j in grid_frontier_targets])
    path = dijkstra(grid_robot_state, set_of_grid_frontier_targets, map_2d)
    
    if path is not None:
        path = np.array(path)
        print(f"Path Returned: {path}")
        return path, frontiers
    else:
        return None, frontiers

def viz_map(map: AdvancedMap):
    fig, ax = plt.subplots(1, 2)
    map.visualize(ax[0])
    map.visualize_points(ax[1])
    plt.show()


if __name__ == "__main__":

    ## TODO: Will update directory structure soon
    scene_name = 'frontier_exploration_test_more_frontiers_v4'
    map_save_dir = f'saves/scenes/{scene_name}'

    # Initialization
    robot = Robot(connection='client')
    
    scan, _ = robot.read_lidar_updated(manual_verification=True, wait_for_updated_reading=True)

    # Initialize Maps

    # Advanced Map
    advanced_map = AdvancedMap()
    advanced_map.init_map(initial_scan=scan)

    advanced_map.visualize(plt.gca())
    plt.show()

    i=0

    # Initialize Starting State
    advanced_map_state = np.array([0.0, 0.0, 0.0])

    while True:
        advanced_map.inflate_obstacles()
        path, frontiers = get_path_to_frontier_robust(advanced_map, advanced_map_state[:2])
    
        advanced_map.visualize_points(plt.gca())
        plt.scatter(frontiers[:, 0], frontiers[:, 1])

        world_coord_path = advanced_map.batch_grid_to_approx_world_coords(path)

        plt.scatter(world_coord_path[:, 0], world_coord_path[:, 1])
        plt.show()

        short_horizon_goal = world_coord_path[min(40, len(world_coord_path)-1)]

        motion_commands = robot.path_to_motion_commands([advanced_map_state, np.array([short_horizon_goal[0], short_horizon_goal[1], 0.0])])
        print(f"Motion Commands: {motion_commands[:2]}")
        input("continue??") # TODO: Remove Later
        for motion_command in motion_commands[:2]:
            motion_type, motion_dist = motion_command
            print(f"Running Motion Command: {motion_command}")
            if abs(motion_dist) < 0.09:
                print("Skipping Motion")
                continue

            m = robot.command_motion_trial(motion_command)
            print(f"Returned m: {m}")
            advanced_map_predicted_state = robot.predict_state(advanced_map_state, m)
            scan, _ = robot.read_lidar_updated(manual_verification=True, wait_for_updated_reading=True)
            advanced_map_updated_state = advanced_map.update(scan, advanced_map_predicted_state)
            advanced_map_state = advanced_map_updated_state
            viz_map(advanced_map)

        i += 1

        # Save Map
        advanced_map.save(map_save_dir, file_name_ext=f"step_{i}")
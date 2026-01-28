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

def get_frontier_target(frontiers, advanced_map_state):
    kd_tree = KDTree(data=frontiers)
    dist, idx = kd_tree.query(advanced_map_state[:2].reshape(1, -1), k=1)
    optimal_frontier = frontiers[idx[0]]
    return optimal_frontier

def bfs(start, target, grid):
    N, M = grid.shape

    sx, sy = start
    q = [(sx, sy)]

    visited = set()

    neighbors = [(0,1), (1,0), (0,-1), (-1,0)]

    child_to_parent = {(sx, sy) : None}

    while q:
        x, y = q.pop(0)

        if (x, y) in visited:
            continue
        visited.add((x,y))

        if (x, y) == (target[0], target[1]):
            print("found_target")
            break

        for ox, oy in neighbors:
            nx = x + ox
            ny = y + oy

            if nx >= 0 and nx < N and ny >= 0 and ny < M and grid[nx, ny] <= 0.5:
                q.append((nx, ny))
                child_to_parent[(nx,ny)] = (x,y)

    target_tuple = (target[0], target[1])

    current = target_tuple
    path = [current]
    while current:
        path.append(current)
        current = child_to_parent[current]
    
    return path[::-1]
    
def dijkstra(start, target, grid):
    N, M = grid.shape

    sx, sy = start
    q = []
    heappush(q, ((0.0, None, (sx, sy))))

    visited = set()

    neighbors = [(0,1), (1,0), (0,-1), (-1,0)]

    child_to_parent = {(sx, sy) : None}

    while q:
        cost, parent, (x, y) = heappop(q)

        if (x, y) in visited:
            continue
        visited.add((x,y))
        child_to_parent[(x,y)] = parent

        if (x, y) == (target[0], target[1]):
            print("found_target")
            break

        for ox, oy in neighbors:
            nx = x + ox
            ny = y + oy

            if nx >= 0 and nx < N and ny >= 0 and ny < M and grid[nx, ny] <= 0.65:
                heappush(q, ((cost + grid[nx, ny], (x, y), (nx, ny))))
                

    target_tuple = (target[0], target[1])

    current = target_tuple
    path = []
    while current:
        path.append(current)
        current = child_to_parent[current]
    
    return path[::-1]
        


        
# TODO: Use dijkstra's instead with cell prob as cost
def get_path_to_frontier(map: AdvancedMap, robot_state: np.ndarray, frontier_target: np.ndarray):
    assert robot_state.shape[0] == 2
    assert frontier_target.shape[0] == 2

    map_2d = map.get_inflated_map_2d()

    grid_robot_state = map.world_to_grid_coords(robot_state[:2])
    grid_frontier_target = map.world_to_grid_coords(frontier_target)

    path = dijkstra(grid_robot_state, grid_frontier_target, map_2d)
    path = np.array(path)
    return path

def viz_map(map: AdvancedMap):
    fig, ax = plt.subplots(1, 2)
    map.visualize(ax[0])
    map.visualize_points(ax[1])
    plt.show()


if __name__ == "__main__":

    ## TODO: Will update directory structure soon
    scene_name = 'frontier_exploration_test'
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
        frontiers = advanced_map.get_frontiers()
        optimal_frontier = get_frontier_target(frontiers, advanced_map_state)
        print(f"Optimal Frontier: {optimal_frontier}")

        path = get_path_to_frontier(advanced_map, advanced_map_state[:2], optimal_frontier[0])
    
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

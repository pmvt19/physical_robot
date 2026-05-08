import numpy as np
import pickle
import matplotlib.pyplot as plt

from physical_robot.robot import Robot
from physical_robot.maps import Map, AdvancedMap, SemanticMap
from physical_robot.robot.robot_space import PhysicalRobotSpace
from physical_robot.models.vlm.vlm_client import VLMClient
from physical_robot.models.vlm.vlm_output_schema import UserSemanticTarget, UserPoseTarget
from physical_robot.models.vlm.prompts import EXTRACT_SEMANTIC_TARGETS, EXTRACT_POSE_TARGET
from heapq import *
from physical_robot.utils import create_local_mask

class TaskPlanner():
    def __init__(self, robot: Robot, map: Map):
        self.robot: Robot = robot
        self.map: Map = map
        self.vlm_client = VLMClient()
        self.robot_space: PhysicalRobotSpace = PhysicalRobotSpace(map_obj=self.map)
        

    def get_target_pose(self, target_theta: float = 0.0):
        x_range, y_range = self.map.get_map_range()
        while True:
            user_input = print("Please provide the X and Y target coordinates")
            vlm_response = self.vlm_client.text_query(EXTRACT_POSE_TARGET.format(user_input,
                                                                                 x_range[0],
                                                                                 x_range[1],
                                                                                 y_range[0],
                                                                                 y_range[1]),
                                                                                UserPoseTarget.model_json_schema())
            user_pose_target = UserPoseTarget.model_validate_json(vlm_response)

            if user_pose_target.valid:
                break
            else:
                print("Unable to extract location information from input")
            
        target = self.robot_space.make_state(np.array([user_pose_target.x, user_pose_target.y, user_pose_target.theta]))
        print(f"Assigned Target State: {np.round(target.value, 2)}")
        return target

class FrontierTaskPlanner(TaskPlanner):
    def __init__(self, robot: Robot, advanced_map: AdvancedMap):
        assert (isinstance(advanced_map, AdvancedMap)), "FrontierTaskPlanner Requires an Advanced Map"
        super().__init__(self, robot, map=advanced_map)
        self.map: AdvancedMap = advanced_map

    def set_starting_pose(self, start: np.ndarray):
        self.start = start
    
    def dijkstra(self, targets, grid):
        N, M = grid.shape

        sx, sy, *_ = self.start
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
    
    def get_target_pose(self, target_theta: float = 0.0):
        map_2d = self.map.get_inflated_map_2d()

        grid_robot_state = self.map.world_to_grid_coords(self.start[:2])

        # Clear start radius
        metric_radius = 100
        grid_radius = metric_radius / self.map.resolution
        local_circle_mask = create_local_mask(map_2d.shape, grid_robot_state, grid_radius)
        map_2d[local_circle_mask] = 0 # TODO: Make a copy??

        frontiers = self.map.get_frontiers()
        grid_frontier_targets = self.map.batch_world_to_grid_coords(frontiers)
        set_of_grid_frontier_targets = set([(i,j) for i,j in grid_frontier_targets])

        self.path_to_target = self.dijkstra(grid_robot_state, set_of_grid_frontier_targets, map_2d)

        if self.path_to_target is not None:
            frontier_goal_state_grid_coords = self.path_to_target[-1]
            frontier_goal_state = self.map.grid_to_approx_world_coords(frontier_goal_state_grid_coords)
            return self.robot_space.make_state(np.array([frontier_goal_state[0], frontier_goal_state[1], target_theta]))
        else:
            return None 
    
    def get_path_to_target(self):
        return self.path_to_target

class SemanticTaskPlanner(TaskPlanner):
    def __init__(self, robot: Robot, semantic_map: SemanticMap):
        assert (isinstance(semantic_map, SemanticMap)), "SemanticTaskPlanner Requires a Semantic Map"
        super().__init__(self, robot=robot, map=semantic_map)
        self.map: SemanticMap = semantic_map
    
    def get_target_pose_from_semantics(self,
                                        layer: str,
                                        item: str,
                                        target_theta: float = 0.0):

        # Filter Semantics to only the layer of interest: room or object
        layer_semantics = self.map.flood_filled_map[:, self.map.layer_name_to_idx[layer]]

        # Get Item Id of item
        item_id = -1
        if layer == 'room':
            item_id = self.map.room_to_id[item]
        elif layer == 'object':
            item_id = self.map.object_to_id[item]
        else:
            raise NotImplementedError

        # Create Vertices Mask for Vertices Corresponding to Specificed item
        selected_item_mask = layer_semantics == item_id

        # Get Vertices Corresponding to Specified Item
        # selected_item_vertices = vertices[selected_item_mask] #TODO: DELETE THIS IF NEXT LINE IS GOOD
        selected_item_vertices_grid_coords = np.where(selected_item_mask == True) # TODO: CHECK THIS

        # Randomly Choose a Vertex from the list of remaining options
        target_pos_idx = np.random.choice(len(selected_item_vertices_grid_coords))
        target_pos_grid_coords = selected_item_vertices_grid_coords[target_pos_idx]
        target_pos = self.map.grid_to_approx_world_coords(target_pos_grid_coords)

        # Create Robot State
        target = self.robot_space.make_state(np.array([target_pos[0], target_pos[1], target_theta]))
        print(f"Assigned Target State: {np.round(target.value, 2)}")
        return target
    
    def get_semantic_target_from_user(self):
        while True:
            user_input = input("Please provide where you want the robot to travel (object or room)\n")
            vlm_response = self.vlm_client.text_query(EXTRACT_SEMANTIC_TARGETS.format(user_input, 
                                                                                self.map.get_room_list(),
                                                                                self.map.get_object_list(),
                                                                                self.map.get_invalid_object_list()), 
                                                                                UserSemanticTarget.model_json_schema())
            user_semantic_target = UserSemanticTarget.model_validate_json(vlm_response)

            if user_semantic_target.valid:
                break
            else:
                print(f"Unable to extract semantic information from input.\n \
                    Reasoning:\n{user_semantic_target.reason} \nPlease Try Again!\n")
        print(user_semantic_target.reason)
        return user_semantic_target.semantic_level, user_semantic_target.item_name
    
    def get_target_pose(self):
        semantic_layer, item = self.get_semantic_target_from_user()
        return self.get_target_pose_from_semantics(semantic_layer.lower(), item.lower())
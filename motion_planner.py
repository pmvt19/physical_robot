import numpy as np

from robot import Robot
from map import Map
from semantic_map import SemanticMap
from robot_utils import localize_robot, mpc_plan_and_follow_trajectory
from robot_space import PhysicalRobotSpace
from motion_planning.prm import PRM
from vlm_client import VLMClient
from icp import run_icp
from utils import transformation_mat_to_state
from vlm_output_schema import UserSemanticTarget
from prompts import EXTRACT_SEMANTIC_TARGETS

def visualize_prm(robot: PhysicalRobotSpace, prm: PRM, path=None):
    # Draw Path
    robot.map.visualize_points(plt.gca())
    prm.draw(plt.gca(), path=path, show_task=True)
    plt.show()

def is_close(state1, state2, threshold=100):
    return np.linalg.norm(state1[:2] - state2[:2]) < threshold

def mpc_plan_and_follow_trajectory(robot: PhysicalRobotSpace,
                                   pf: ParticleFilter,
                                   map: Map,
                                   prm: PRM,
                                   start: NumpyState,
                                   target: NumpyState,
                                   visualize_iterative_path: bool = False):

    path = prm.search(start, target)
    visualize_prm(robot, prm, path)
    path = [p.value for p in path]
    current_state = start

    while not is_close(current_state.value, target.value):
        motion_commands = robot.path_to_motion_commands(path)
        
        for i, motion_command in enumerate(motion_commands):
            print(f"Executing Motion Command: {motion_command}")
            m, predicted_state = robot.command_motion_and_predict_state(current_state.value, motion_command)
            
            coords, lidar_data = robot.read_lidar_updated(wait_for_updated_reading=True, manual_verification=False)

            # Particle Filter State Prediction Update
            current_state = pf.step(motion_delta=motion_command, scan=lidar_data)
            print(f"Current State After Particle Filter Update: {np.round(current_state, 2)}")

            # ICP State Refinement
            T = run_icp(coords, map.get_points(), current_state, filter_init_outliers=False, visualize=False)
            current_state = robot.make_state(transformation_mat_to_state(T))
            print(f"Current State After ICP Refinement: {np.round(current_state.value, 2)}")

            # Break to replan if we are on the 4th Motion Command
            if i == 3:
                break
        
        path = prm.search(current_state, target)
        if visualize_iterative_path:
            visualize_prm(robot, prm, path)
        path = [p.value for p in path]
        motion_commands = robot.path_to_motion_commands(path)

class MotionPlanner():
    def __init__(self, robot: Robot, map: Map):
        self.robot: Robot = robot
        self.map: Map = map

        self.robot_space: PhysicalRobotSpace = PhysicalRobotSpace(map_obj=self.map)

        self.path_lookahead_distance = 3

    def localize_robot(self):
        self.start, self.pf = localize_robot(self.robot, self.map)

    def search(self):
        raise NotImplementedError

    def get_target_from_user(self):
        # TODO: Implement robust function for user to tell where the robot should go
        return None

    def run_motion_planning(self):
        # Localize the Robot
        self.localize_robot()

        # Get Target From User
        target = self.get_target_from_user()

        # Convert Target to NumpyState
        target = self.robot_space.make_state(target)

        # Plan a path to the Target and Follow with MPC
        mpc_plan_and_follow_trajectory(self.robot_space, self.pf, self.map, self.prm, self.robot_space.make_state(self.start), target)

    def move_to_target(self, target):
        while not is_close(current_state.value, target.value):
            motion_commands = self.robot.path_to_motion_commands(path)
            self.step_motion_execution(current_state, motion_commands)

        path = self.search(current_state.value, target.value)
        numpy_path = [p.value for p in path]
        motion_commands = self.robot.path_to_motion_commands(numpy_path)

    def step_motion_execution(self, current_state, motion_commands):
        for i, motion_command in enumerate(motion_commands):
            print(f"Executing Motion Command: {motion_command}")
            m, predicted_state = self.robot.command_motion_and_predict_state(current_state.value, motion_command)
            
            coords, lidar_data = self.robot.read_lidar_updated(wait_for_updated_reading=True, manual_verification=False)

            # Particle Filter State Prediction Update
            current_state = self.pf.step(motion_delta=motion_command, scan=lidar_data)
            print(f"Current State After Particle Filter Update: {np.round(current_state, 2)}")

            # ICP State Refinement
            T = run_icp(coords, self.map.get_points(), current_state, filter_init_outliers=False, visualize=False)
            current_state = self.robot_space.make_state(transformation_mat_to_state(T))
            print(f"Current State After ICP Refinement: {np.round(current_state.value, 2)}")

            # Break to replan if we are on the 4th Motion Command
            if i == self.path_lookahead_distance:
                break

        return current_state

"""
The original idea behind grid based motion planner was to allow for motion 
planning within the map object itself. However, the grid motion planner is 
mostly used for frontier exploration at the momement. In this case, it creates
a path, looks ahead maybe only 40 steps, and drives straight to that point. This
will continuously repeat. I'm not sure how to implement that in the same way here.
This motion planner might not be usable while maps are being built.
"""
class GridMotionPlanner(MotionPlanner):
    def __init__(self, robot: Robot, map: Map):
        super().__init__(robot=robot, map=map)
    
    def search(self, start, target):
        current_state_grid_coords = self.map.world_to_grid_coords(start.value[:2])
        target_grid_coords = self.map.world_to_grid_coords(target.value[:2])
        grid_coords_path = self.map.dijkstra(current_state_grid_coords, target_grid_coords)
        world_coord_path = self.map.batch_grid_to_approx_world_coords(grid_coords_path)
        return world_coord_path

class PrmMotionPlanner(MotionPlanner):
    def __init__(self, robot: Robot, map: Map):
        super().__init__(robot=robot, map=map)

        self.path_lookahead_distance = 3
    
    def build_prm(self):
        self.prm = PRM(env=self.robot_space, num_samples=10000, num_neighbors=10, validate_edges=True)
        self.prm.create_graph()
    
    def search(self, start, target):
        prm_path = self.prm.search(start.value, target.value)
        prm_path = [p.value for p in prm_path]
        return prm_path

FREE_THRESHOLD = 0.5
class SemanticMotionPlanner(PrmMotionPlanner):
    def __init__(self, robot: Robot, semantic_map: SemanticMap):
        assert (isinstance(semantic_map, SemanticMap))
        self.robot: Robot = robot
        self.map: SemanticMap = semantic_map

        self.robot_space: PhysicalRobotSpace = PhysicalRobotSpace(map_obj=self.map)

        self.vlm_client = VLMClient()

    def get_semantic_labeled_prm_vertices(self):
        # Get only X,Y values from vertices
        vertices = self.prm.graph.vertices[:, :2]

        # Convert to grid coords and get geometric map values at the coordinate positions
        grid_coord_vertices = self.map.batch_world_to_grid_coords(vertices)
        vertex_values = self.map.batch_get_value_at_grid_coords(grid_coord_vertices)

        # Mask for Vertices that fall in unoccupied cells
        unoccupied_vertices_mask = vertex_values < FREE_THRESHOLD

        # Filtering to vertices that fall in unoccupied cells
        unoccupied_vertices = vertices[unoccupied_vertices_mask]

        # Converting unoccupied vertices to grid coords
        unoccupied_vertices_grid_coords = self.map.batch_world_to_grid_coords(unoccupied_vertices)

        # Get Flood Filled Semantic Values for vertices in unoccupied cells
        unoccupied_vertices_semantics = self.map.batch_get_semantic_flood_fill_value_at_grid_coords(unoccupied_vertices_grid_coords)

        # Mask for Unoccupied Vertices which have Semantic Labels
        unoccupied_vertices_labeled_semantics_mask = np.sum(unoccupied_vertices_semantics, axis=1) > 0

        # Get unoccupied vertices that have labeled semantic (not the semantics)
        unoccupied_vertices_with_labeled_semantics = unoccupied_vertices[unoccupied_vertices_labeled_semantics_mask]

        # Get unoccupied vertices semantics (not the coords)
        unoccupied_vertices_labeled_semantics = unoccupied_vertices_semantics[unoccupied_vertices_labeled_semantics_mask]

        return unoccupied_vertices_with_labeled_semantics, unoccupied_vertices_labeled_semantics
    
    def get_target_pose_from_semantics(self,
            robot: PhysicalRobotSpace, 
                              semantic_map: SemanticMap, 
                              vertices: np.ndarray, 
                              semantics: np.ndarray, 
                              layer: str, 
                              item: str, 
                              target_theta: float = 0.0):

        # Filter Semantics to only the layer of interest: room or object
        layer_semantics = semantics[:, semantic_map.layer_name_to_idx[layer]]

        # Get Item Id of item
        item_id = -1
        if layer == 'room':
            item_id = semantic_map.room_to_id[item]
        elif layer == 'object':
            item_id = semantic_map.object_to_id[item]
        else:
            raise NotImplementedError

        # Create Vertices Mask for Vertices Corresponding to Specificed item
        selected_item_mask = layer_semantics == item_id

        # Get Vertices Corresponding to Specified Item
        selected_item_vertices = vertices[selected_item_mask]

        # Randomly Choose a Vertex from the list of remaining options
        target_pos_idx = np.random.choice(len(selected_item_vertices))
        target_pos = selected_item_vertices[target_pos_idx]

        # Create Robot State
        target = robot.make_state(np.array([target_pos[0], target_pos[1], target_theta]))
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

    def run_motion_planning(self):
        # Flood Fill the Semantic Map
        self.map.flood_fill(limit_fill_extent=False, method='bfs')

        # Get Semantic Target Layer and Item from User Natural Language Input
        semantic_layer, item = self.get_semantic_target_from_user()

        # Create and Build PRM
        self.build_prm()

        # Label Vertices in PRM with Semantic Information
        semantic_vertices, semantic_labels = self.get_semantic_labeled_prm_vertices()

        # Get Target State Based on User Semantic Input 
        target = self.get_target_from_semantics(semantic_vertices, semantic_labels, semantic_layer.lower(), item)

        # Localize Robot within Map
        self.localize_robot()

        # Plan and Follow path with MPC
        mpc_plan_and_follow_trajectory(self.robot_space, self.pf, self.map, self.prm, self.robot_space.make_state(self.start), target)
    
    def move_to_semantic_target(self, semantic_target: UserSemanticTarget):
        assert (semantic_target.valid), "Semantic Target is Not Valid"
        semantic_level, item_name = semantic_target.semantic_level, semantic_target.item_name

        # Label Vertices in PRM with Semantic Information
        semantic_vertices, semantic_labels = self.get_semantic_labeled_prm_vertices()
        target = self.get_target_pose_from_semantics(semantic_vertices, semantic_labels, semantic_layer.lower(), item)

        self.move_to_target(target)

